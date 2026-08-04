import sys, json, math
from pathlib import Path
import numpy as np

from isaacsim import SimulationApp
sim = SimulationApp({"headless": True, "width": 1280, "height": 720})

import omni.usd
import omni.replicator.core as rep
from pxr import UsdGeom, UsdLux, UsdPhysics, Gf, Sdf

args = sys.argv[1:]
# resolve tudo relativo à PASTA DESTE SCRIPT -> funciona em container OU nativo, em qualquer path
HERE = Path(__file__).resolve().parent
fname = next((a for a in args if a.endswith((".usd", ".usdz", ".usda"))), "lab_v4_robot_scene.usdz")
USD = fname if Path(fname).is_absolute() else str(HERE / fname)
STEPS = int(args[args.index("--steps") + 1]) if "--steps" in args else 3000
ROBOT = args[args.index("--robot") + 1] if "--robot" in args else "carter"
CAM = args[args.index("--cam") + 1] if "--cam" in args else "follow"
OUT = str(HERE / "robot_nav")
NAV = str(HERE / "nav_path.json")

try:
    from isaacsim.core.utils.stage import open_stage, add_reference_to_stage
    from isaacsim.core.api import World
    from isaacsim.storage.native import get_assets_root_path
except Exception:
    from omni.isaac.core.utils.stage import open_stage, add_reference_to_stage
    from omni.isaac.core import World
    from omni.isaac.core.utils.nucleus import get_assets_root_path


def log(m): print(f"[nav] {m}", flush=True)


# ── asset do robô (path varia por versão -> tenta vários) ────────────────────
ASSETS = get_assets_root_path()
CANDS = {
    "carter": ["/Isaac/Robots/Carter/nova_carter/nova_carter.usd",
               "/Isaac/Robots/Carter/carter_v1.usd",
               "/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd"],
    "jetbot": ["/Isaac/Robots/Jetbot/jetbot.usd"],
}


def robot_asset():
    import omni.client
    for rel in CANDS.get(ROBOT, []):
        url = ASSETS + rel
        try:
            if omni.client.stat(url)[0] == omni.client.Result.OK:
                return url
        except Exception:
            pass
    return ASSETS + CANDS["carter"][0]      # tenta na mesma


def main():
    open_stage(USD)
    stage = omni.usd.get_context().get_stage()
    world = World(stage_units_in_meters=1.0)

    # física + chão plano de segurança (o mesh já colide; o plano evita cair em buracos)
    UsdPhysics.Scene.Define(stage, Sdf.Path("/World/physicsScene"))
    from isaacsim.core.api.objects import GroundPlane
    try:
        GroundPlane(prim_path="/World/groundSafety", z_position=0.0, size=30.0)
    except Exception as e:
        log(f"ground plane fallback: {e}")
    UsdLux.DomeLight.Define(stage, "/World/NavLight").CreateIntensityAttr(1000.0)

    # waypoints
    nav = json.loads(Path(NAV).read_text())
    WP = np.array(nav["waypoints"], float)
    log(f"{len(WP)} waypoints, start={nav['start']} goal={nav['goal']} ({nav.get('path_len_m','?')}m)")

    # ── robô ──
    from isaacsim.robot.wheeled_robots.robots import WheeledRobot
    from isaacsim.robot.wheeled_robots.controllers.differential_controller import DifferentialController
    asset = robot_asset(); log(f"robô: {asset}")
    start = WP[0]
    add_reference_to_stage(asset, "/World/Robot")

    # auto-deteta os 2 joints das rodas (nomes variam por versão/robô)
    wheel_joints = []
    for p in stage.Traverse():
        tn = p.GetTypeName()
        nm = p.GetName().lower()
        if "RevoluteJoint" in str(tn) and ("wheel" in nm or "drive" in nm) \
           and str(p.GetPath()).startswith("/World/Robot"):
            wheel_joints.append(p.GetName())
    left = [j for j in wheel_joints if "left" in j.lower() or j.lower().endswith("l")]
    right = [j for j in wheel_joints if "right" in j.lower() or j.lower().endswith("r")]
    if left and right:
        wnames = [left[0], right[0]]
    elif len(wheel_joints) >= 2:
        wnames = wheel_joints[:2]
    else:
        wnames = ["joint_wheel_left", "joint_wheel_right"]     # fallback Carter
    log(f"joints de roda detetados: {wheel_joints} -> a usar {wnames}")

    robot = world.scene.add(WheeledRobot(
        prim_path="/World/Robot", name="robot", create_robot=False,
        wheel_dof_names=wnames,
        position=np.array([start[0], start[1], 0.15])))
    ctrl = DifferentialController(name="diff", wheel_radius=0.14, wheel_base=0.41)

    # câmara p/ gravar
    cam = rep.create.camera()
    rp = rep.create.render_product(cam, (1280, 720))
    writer = rep.WriterRegistry.get("BasicWriter")
    writer.initialize(output_dir=OUT, rgb=True)
    writer.attach([rp])

    world.reset()
    try:
        log(f"DOFs do robô: {list(robot.dof_names)}")
    except Exception:
        pass
    log("simulação iniciada")

    wp_i = 1
    frame = 0
    RENDER_EVERY = 15          # grava 1 frame a cada 15 passos de física (mais rápido)
    PROG = 50                  # log de progresso a cada 50 passos
    last_xy = (start[0], start[1])
    for step in range(STEPS):
        pos, quat = robot.get_world_pose()
        yaw = math.atan2(2*(quat[0]*quat[3]+quat[1]*quat[2]),
                         1-2*(quat[2]**2+quat[3]**2))
        tgt = WP[min(wp_i, len(WP)-1)]
        dx, dy = tgt[0]-pos[0], tgt[1]-pos[1]
        dist = math.hypot(dx, dy)
        if dist < 0.35:                        # chegou ao waypoint
            if wp_i < len(WP)-1:
                wp_i += 1
            elif dist < 0.25:
                log(f"CHEGOU AO GOAL em {step} passos"); break
        # pure-pursuit: vira p/ o alvo, avança
        ang = math.atan2(dy, dx) - yaw
        ang = (ang + math.pi) % (2*math.pi) - math.pi
        v = 0.6 * max(0.15, math.cos(ang))     # abranda em curva
        w = 1.8 * ang
        robot.apply_wheel_actions(ctrl.forward(command=[v, w]))
        # física a cada passo (rápido); render/gravação só a cada RENDER_EVERY
        if step % RENDER_EVERY == 0:
            if CAM == "top":
                eye = (pos[0], pos[1], 8.0); look = (pos[0], pos[1], 0.0)
            else:
                eye = (pos[0]-1.8*math.cos(yaw), pos[1]-1.8*math.sin(yaw), 1.4); look = tuple(pos)
            with cam:
                rep.modify.pose(position=eye, look_at=look)
            rep.orchestrator.step(rt_subframes=4); frame += 1
        else:
            world.step(render=False)
        # progresso visível a cada PROG passos
        if step % PROG == 0:
            log(f"passo {step}/{STEPS} | robô=[{pos[0]:.2f},{pos[1]:.2f}] "
                f"wp {wp_i}/{len(WP)-1} dist={dist:.2f}m frames={frame}")
            if step > 0 and math.hypot(pos[0]-last_xy[0], pos[1]-last_xy[1]) < 0.02:
                log("  aviso: robô quase não se moveu (verificar rodas/física)")
            last_xy = (pos[0], pos[1])
    rep.orchestrator.wait_until_complete()
    endp, _ = robot.get_world_pose()
    log(f"fim: robô em [{endp[0]:.2f},{endp[1]:.2f}], {frame} frames em {OUT}/")
    print("ROBOT NAV DONE")


if __name__ == "__main__":
    try:
        main()
    finally:
        sim.close(); sys.exit(0)
