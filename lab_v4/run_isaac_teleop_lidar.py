"""
Teleop + LIDAR 2D no digital twin. Igual ao run_isaac_teleop.py (conduz por /cmd_vel via UDP),
mas também simula um lidar 2D por raycasting contra o mesh de colisão e ENVIA o scan por UDP
para o nó isaac_scan_bridge.py (que publica /scan + TF p/ o RViz2).

Fluxo:  cmd_vel (UDP 9091, entra) -> conduz ;  scan+pose (UDP 9092, sai) -> ROS /scan + TF

Uso (Isaac 5.x nativo, SEM source do ROS):
  <isaac>/python.sh /caminho/lab_v4/run_isaac_teleop_lidar.py
Opções: --speed 1.0  --rays 360  --rings 1  --vfov 30  --lidar-hz 10  --range 12
  --rays  : nº de raios em azimute (denso; 360 por omissão)
  --rings : camadas verticais. 1 = lidar 2D (/scan). >1 = lidar 3D (/scan + /points PointCloud2)
  --vfov  : abertura vertical em graus (só 3D). Ex.: --rings 16 --vfov 30 = tipo lidar 3D.
Noutros terminais (ROS sourced): cmd_vel_udp_bridge.py  +  isaac_scan_bridge.py  +  rviz2
"""
import sys, socket, math, struct
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
args = sys.argv[1:]
HEADLESS = "--headless" in args
SPEED = float(args[args.index("--speed") + 1]) if "--speed" in args else 1.0
PORT_CMD = int(args[args.index("--port") + 1]) if "--port" in args else 9091
PORT_SCAN = int(args[args.index("--scan-port") + 1]) if "--scan-port" in args else 9092
NRAYS = int(args[args.index("--rays") + 1]) if "--rays" in args else 360      # azimute (mais denso)
NRINGS = int(args[args.index("--rings") + 1]) if "--rings" in args else 1      # 1=2D; >1=3D
VFOV = float(args[args.index("--vfov") + 1]) if "--vfov" in args else 30.0     # FOV vertical (graus, 3D)
LIDAR_HZ = float(args[args.index("--lidar-hz") + 1]) if "--lidar-hz" in args else 10.0
RANGE = float(args[args.index("--range") + 1]) if "--range" in args else 12.0
LIDAR_Z = 0.35   # altura do lidar acima da base do robô
fname = next((a for a in args if a.endswith((".usd", ".usdz", ".usda"))), "lab_v4_robot_scene.usdz")
USD = fname if Path(fname).is_absolute() else str(HERE / fname)

from isaacsim import SimulationApp
sim = SimulationApp({"headless": HEADLESS, "width": 1280, "height": 720})

import omni.usd
from pxr import UsdPhysics, UsdLux, Sdf
from omni.physx import get_physx_scene_query_interface

try:
    from isaacsim.core.utils.stage import open_stage, add_reference_to_stage
    from isaacsim.core.api import World
    from isaacsim.storage.native import get_assets_root_path
except Exception:
    from omni.isaac.core.utils.stage import open_stage, add_reference_to_stage
    from omni.isaac.core import World
    from omni.isaac.core.utils.nucleus import get_assets_root_path


def log(m): print(f"[lidar] {m}", flush=True)


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
    add_reference_to_stage(robot_asset(), "/World/Robot")
    wj = [p.GetName() for p in stage.Traverse()
          if "RevoluteJoint" in str(p.GetTypeName()) and "wheel" in p.GetName().lower()
          and str(p.GetPath()).startswith("/World/Robot")]
    left = [j for j in wj if "left" in j.lower()]; right = [j for j in wj if "right" in j.lower()]
    wnames = [left[0], right[0]] if (left and right) else (wj[:2] or ["joint_wheel_left", "joint_wheel_right"])

    spawn = np.array([0.0, 0.0, 0.20])
    npj = HERE / "nav_path.json"
    if npj.exists():
        import json
        s = json.loads(npj.read_text()).get("start")
        if s:
            spawn = np.array([s[0], s[1], 0.20])
    robot = world.scene.add(WheeledRobot(
        prim_path="/World/Robot", name="robot", create_robot=False,
        wheel_dof_names=wnames, position=spawn))
    ctrl = DifferentialController(name="diff", wheel_radius=0.14, wheel_base=0.41)
    world.reset()

    try:                                      # rodas em modo velocidade
        idxs = [robot.get_dof_index(n) for n in wnames]
        ac = robot.get_articulation_controller()
        kps, kds = ac.get_gains()
        kps = np.array(kps, float); kds = np.array(kds, float)
        for i in idxs:
            kps[i] = 0.0; kds[i] = 1.0e5
        ac.set_gains(kps, kds)
    except Exception as e:
        log(f"aviso ganhos: {e}")

    # sockets: entra cmd_vel (9091), sai scan (9092)
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); rx.bind(("127.0.0.1", PORT_CMD)); rx.setblocking(False)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); scan_addr = ("127.0.0.1", PORT_SCAN)
    modo = "3D" if NRINGS > 1 else "2D"
    log(f"cmd_vel<-UDP {PORT_CMD} | scan->UDP {PORT_SCAN} "
        f"(lidar {modo}: {NRINGS}anel×{NRAYS}az, {LIDAR_HZ:.0f}Hz, {RANGE:.0f}m)")

    sq = get_physx_scene_query_interface()
    amin, amax = -math.pi, math.pi
    ainc = (amax - amin) / NRAYS
    OFF = 0.40          # começa o raio FORA do corpo do robô (evita auto-colisão)
    # elevações dos anéis (3D). 1 anel -> [0] (lidar 2D horizontal)
    if NRINGS <= 1:
        elevs = [0.0]
    else:
        vr = math.radians(VFOV)
        elevs = [(-vr / 2) + k * (vr / (NRINGS - 1)) for k in range(NRINGS)]

    def do_scan(px, py, pz, yaw):
        """Devolve ranges achatados (anel-maior: anel0 az0..azN, anel1 ...)."""
        rr = []
        oz = pz + LIDAR_Z
        for el in elevs:
            ce, se = math.cos(el), math.sin(el)
            for i in range(NRAYS):
                wa = yaw + amin + i * ainc
                cx, cy = math.cos(wa) * ce, math.sin(wa) * ce
                ox, oy, ozr = px + OFF * cx, py + OFF * cy, oz + OFF * se
                hit = sq.raycast_closest((ox, oy, ozr), (cx, cy, se), RANGE - OFF)
                rr.append(OFF + float(hit["distance"]) if (hit and hit.get("hit")) else RANGE)
        return rr

    v = w = 0.0; stale = 0; step = 0
    period = max(1, int(60.0 / LIDAR_HZ))
    while sim.is_running():
        got = False
        while True:
            try:
                data, _ = rx.recvfrom(64); v, w = map(float, data.decode().split(",")); got = True
            except BlockingIOError:
                break
            except Exception:
                break
        stale = 0 if got else stale + 1
        if stale > 30:
            v = w = 0.0
        robot.apply_wheel_actions(ctrl.forward(command=[v * SPEED, w * SPEED]))
        world.step(render=True)
        step += 1
        if step % period == 0:                       # publica scan
            p, q = robot.get_world_pose()
            yaw = math.atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] ** 2 + q[3] ** 2))
            rr = do_scan(float(p[0]), float(p[1]), float(p[2]), yaw)
            # cabeçalho: pose(4f) + rays(i) rings(i) + vfov(f) range(f) + ranges(rings*rays f)
            pkt = struct.pack("<4f", float(p[0]), float(p[1]), float(p[2]), yaw) \
                + struct.pack("<iiff", NRAYS, NRINGS, VFOV, RANGE) \
                + struct.pack(f"<{len(rr)}f", *rr)
            try:
                tx.sendto(pkt, scan_addr)
            except Exception:
                pass
            if step % (period * 20) == 0:              # debug do scan a cada ~2s
                arr = np.array(rr)
                nhit = int((arr < RANGE - 0.01).sum())
                log(f"scan: {NRINGS}anel×{NRAYS}az hits={nhit}/{len(rr)} "
                    f"dist min={arr.min():.2f} méd={arr.mean():.2f} max={arr.max():.2f}")
    rx.close(); tx.close(); print("LIDAR TELEOP DONE")


if __name__ == "__main__":
    try:
        main()
    finally:
        sim.close(); sys.exit(0)
