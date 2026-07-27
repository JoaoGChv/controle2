"""
Teleop do robô no digital twin (Caminho A v2): Isaac Sim (GUI) carrega a cena 3DGS
(lab_v4_robot_scene.usdz, gravity-aligned) + Nova Carter, e conduz a base a partir de comandos
(v,w) recebidos por UDP — enviados pela ponte cmd_vel_udp_bridge.py (PS4->joy->teleop_twist_joy->
/cmd_vel->UDP). Não usa rclpy (o Python do Isaac é 3.11, incompatível com o rclpy do Humble 3.10).

Uso (Isaac 5.x nativo; NÃO precisa de sourcing do ROS neste terminal):
  <isaac>/python.sh /caminho/lab_v4/run_isaac_teleop.py
Opções: --headless  --speed 1.0  --port 9091  --usd <nome.usdz>
Noutro terminal (sistema, ROS sourced): python3 cmd_vel_udp_bridge.py
"""
import sys, socket
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
args = sys.argv[1:]
HEADLESS = "--headless" in args
SPEED = float(args[args.index("--speed") + 1]) if "--speed" in args else 1.0
PORT = int(args[args.index("--port") + 1]) if "--port" in args else 9091
fname = next((a for a in args if a.endswith((".usd", ".usdz", ".usda"))), "lab_v4_robot_scene.usdz")
USD = fname if Path(fname).is_absolute() else str(HERE / fname)

from isaacsim import SimulationApp
sim = SimulationApp({"headless": HEADLESS, "width": 1280, "height": 720})

import omni.usd
from pxr import UsdPhysics, UsdLux, Sdf

try:
    from isaacsim.core.utils.stage import open_stage, add_reference_to_stage
    from isaacsim.core.api import World
    from isaacsim.storage.native import get_assets_root_path
except Exception:
    from omni.isaac.core.utils.stage import open_stage, add_reference_to_stage
    from omni.isaac.core import World
    from omni.isaac.core.utils.nucleus import get_assets_root_path


def log(m): print(f"[teleop] {m}", flush=True)


def robot_asset():
    import omni.client
    A = get_assets_root_path()
    for rel in ["/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd",
                "/Isaac/Robots/Carter/nova_carter/nova_carter.usd"]:
        try:
            if omni.client.stat(A + rel)[0] == omni.client.Result.OK:
                return A + rel
        except Exception:
            pass
    return A + "/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd"


def main():
    open_stage(USD)
    stage = omni.usd.get_context().get_stage()
    world = World(stage_units_in_meters=1.0)

    UsdPhysics.Scene.Define(stage, Sdf.Path("/World/physicsScene"))
    from isaacsim.core.api.objects import GroundPlane
    try:
        GroundPlane(prim_path="/World/groundSafety", z_position=0.0, size=30.0)
    except Exception as e:
        log(f"ground plane: {e}")
    UsdLux.DomeLight.Define(stage, "/World/NavLight").CreateIntensityAttr(1000.0)

    from isaacsim.robot.wheeled_robots.robots import WheeledRobot
    from isaacsim.robot.wheeled_robots.controllers.differential_controller import DifferentialController
    asset = robot_asset(); log(f"robô: {asset}")
    add_reference_to_stage(asset, "/World/Robot")

    wj = [p.GetName() for p in stage.Traverse()
          if "RevoluteJoint" in str(p.GetTypeName())
          and "wheel" in p.GetName().lower()
          and str(p.GetPath()).startswith("/World/Robot")]
    left = [j for j in wj if "left" in j.lower()]; right = [j for j in wj if "right" in j.lower()]
    wnames = [left[0], right[0]] if (left and right) else (wj[:2] or ["joint_wheel_left", "joint_wheel_right"])
    log(f"rodas: {wnames}")

    # nasce num ponto livre conhecido (start do caminho A*, se existir)
    spawn = np.array([0.0, 0.0, 0.20])
    npj = HERE / "nav_path.json"
    if npj.exists():
        import json
        s = json.loads(npj.read_text()).get("start")
        if s:
            spawn = np.array([s[0], s[1], 0.20])
    log(f"spawn do robô: {spawn.round(2)}")
    robot = world.scene.add(WheeledRobot(
        prim_path="/World/Robot", name="robot", create_robot=False,
        wheel_dof_names=wnames, position=spawn))
    ctrl = DifferentialController(name="diff", wheel_radius=0.14, wheel_base=0.41)

    world.reset()

    # controlo por VELOCIDADE nas rodas (stiffness=0, damping alto) — corrige a lentidão
    try:
        idxs = [robot.get_dof_index(n) for n in wnames]
        ac = robot.get_articulation_controller()
        ac.set_gains(kps=np.zeros(len(idxs)), kds=np.full(len(idxs), 1.0e5),
                     joint_indices=np.array(idxs))
        log(f"ganhos de roda p/ velocidade OK (dof idx {idxs})")
    except Exception as e:
        log(f"aviso: não ajustei ganhos ({e}); se andar lento usa --speed 3")

    # ── socket UDP (recebe v,w da ponte) ─────────────────────────────────────
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", PORT))
    sock.setblocking(False)
    log(f"a ouvir UDP 127.0.0.1:{PORT} — corre a ponte cmd_vel_udp_bridge.py e conduz com o PS4")

    v = w = 0.0
    stale = 0
    step = 0
    while sim.is_running():
        # drena o socket -> fica com o comando mais recente
        got = False
        while True:
            try:
                data, _ = sock.recvfrom(64)
                v, w = map(float, data.decode().split(","))
                got = True
            except BlockingIOError:
                break
            except Exception:
                break
        stale = 0 if got else stale + 1
        if stale > 30:                      # sem comando há ~0.5s -> pára (segurança)
            v = w = 0.0
        robot.apply_wheel_actions(ctrl.forward(command=[v * SPEED, w * SPEED]))
        world.step(render=True)
        step += 1
        if step % 120 == 0:
            p, _ = robot.get_world_pose()
            log(f"cmd=[v {v:.2f}, w {w:.2f}] robô=[{p[0]:.2f},{p[1]:.2f}]")
    sock.close()
    print("TELEOP DONE")


if __name__ == "__main__":
    try:
        main()
    finally:
        sim.close(); sys.exit(0)
