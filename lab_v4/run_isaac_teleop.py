"""
Teleop ROS2 do robô no digital twin (Caminho A v2): Isaac Sim COM GUI carrega a cena 3DGS
(lab_v4_robot_scene.usdz, gravity-aligned) + Nova Carter, e conduz a base a partir de /cmd_vel
(geometry_msgs/Twist). O comando PS4 gera /cmd_vel via joy+teleop_twist_joy (do lado ROS2).

Reutiliza o caminho de controlo JÁ validado (WheeledRobot + DifferentialController +
apply_wheel_actions) — só a origem do (v,w) muda: /cmd_vel em vez de waypoints. Subscreve via
rclpy no mesmo processo (fonte ROS2 antes de correr, para o rclpy estar no PYTHONPATH).

Uso (Isaac 5.x nativo; ROS2 Humble já 'sourced'):
  source /opt/ros/humble/setup.bash
  <isaac>/python.sh /caminho/lab_v4/run_isaac_teleop.py
Opções: --headless  --speed 1.0 (multiplicador de velocidade)  --usd <nome.usdz>
"""
import sys, math
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
args = sys.argv[1:]
HEADLESS = "--headless" in args
SPEED = float(args[args.index("--speed") + 1]) if "--speed" in args else 1.0
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


# ── rclpy (precisa de 'source /opt/ros/humble/setup.bash' antes) ─────────────
try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
except Exception as e:
    log(f"ERRO a importar rclpy: {e}")
    log("Faz PRIMEIRO: source /opt/ros/humble/setup.bash   e volta a correr.")
    sim.close(); sys.exit(1)


class CmdVel(Node):
    def __init__(self):
        super().__init__("isaac_teleop")
        self.v = 0.0; self.w = 0.0; self.t = 0.0
        self.create_subscription(Twist, "/cmd_vel", self.cb, 10)
    def cb(self, msg):
        self.v = float(msg.linear.x); self.w = float(msg.angular.z)
        self.t = self.get_clock().now().nanoseconds * 1e-9


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

    # joints de roda (auto-deteta)
    wj = [p.GetName() for p in stage.Traverse()
          if "RevoluteJoint" in str(p.GetTypeName())
          and "wheel" in p.GetName().lower()
          and str(p.GetPath()).startswith("/World/Robot")]
    left = [j for j in wj if "left" in j.lower()]; right = [j for j in wj if "right" in j.lower()]
    wnames = [left[0], right[0]] if (left and right) else (wj[:2] or ["joint_wheel_left", "joint_wheel_right"])
    log(f"rodas: {wnames}")

    robot = world.scene.add(WheeledRobot(
        prim_path="/World/Robot", name="robot", create_robot=False,
        wheel_dof_names=wnames, position=np.array([0.0, 0.0, 0.20])))
    ctrl = DifferentialController(name="diff", wheel_radius=0.14, wheel_base=0.41)

    world.reset()

    # garante CONTROLO POR VELOCIDADE nas rodas (stiffness=0, damping alto)
    # — provável causa da lentidão anterior era as rodas em modo posição.
    try:
        idxs = [robot.get_dof_index(n) for n in wnames]
        ac = robot.get_articulation_controller()
        kps = np.zeros(len(idxs)); kds = np.full(len(idxs), 1.0e5)
        ac.set_gains(kps=kps, kds=kds, joint_indices=np.array(idxs))
        log(f"ganhos de roda p/ velocidade OK (dof idx {idxs})")
    except Exception as e:
        log(f"aviso: não consegui ajustar ganhos ({e}); se andar lento usa --speed 3")

    rclpy.init()
    node = CmdVel()
    log("PRONTO — publica /cmd_vel (joy+teleop_twist_joy) para conduzir. Ctrl-C para sair.")

    step = 0
    while sim.is_running():
        rclpy.spin_once(node, timeout_sec=0.0)
        # timeout de segurança: sem comando há >0.5s -> pára
        now = node.get_clock().now().nanoseconds * 1e-9
        v, w = (node.v, node.w) if (now - node.t) < 0.5 else (0.0, 0.0)
        robot.apply_wheel_actions(ctrl.forward(command=[v * SPEED, w * SPEED]))
        world.step(render=True)
        step += 1
        if step % 120 == 0:
            p, _ = robot.get_world_pose()
            log(f"cmd=[v {v:.2f}, w {w:.2f}] robô=[{p[0]:.2f},{p[1]:.2f}]")
    node.destroy_node(); rclpy.shutdown()
    print("TELEOP DONE")


if __name__ == "__main__":
    try:
        main()
    finally:
        sim.close(); sys.exit(0)
