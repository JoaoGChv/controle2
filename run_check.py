"""Validação headless da física (Isaac Sim 5.0). O cubo deve cair sobre o mesh.
Uso dentro do container:  ./python.sh /root/dtvf_isaac/run_check.py <cena>_isaac_sim.usd
"""
import sys
from isaacsim import SimulationApp
sim = SimulationApp({"headless": True})

USD = "/root/dtvf_isaac/" + (sys.argv[1] if len(sys.argv) > 1 else "video1_isaac_sim.usd")
CUBE = "/World/TestCube"

try:
    from isaacsim.core.api import SimulationContext
    from isaacsim.core.utils.stage import open_stage
except Exception:
    from omni.isaac.core import SimulationContext
    from omni.isaac.core.utils.stage import open_stage
from pxr import UsdGeom, Gf
import omni.usd


def cube_y():
    stage = omni.usd.get_context().get_stage()
    x = UsdGeom.Xformable(stage.GetPrimAtPath(CUBE)).ComputeLocalToWorldTransform(0)
    return Gf.Transform(x).GetTranslation()[1]


def log(m): print(f"[check] {m}", flush=True)


def main():
    log(f"a abrir {USD} ..."); open_stage(USD)
    log("stage aberto. física (cooking do colisor)...")
    import carb
    carb.settings.get_settings().set_bool("/physics/updateToUsd", True)
    carb.settings.get_settings().set_bool("/physics/updateVelocitiesToUsd", True)
    ctx = SimulationContext(stage_units_in_meters=1.0); ctx.initialize_physics()
    y0 = cube_y(); log(f"Y inicial = {y0:.3f}. 240 passos..."); ctx.play()
    for i in range(240):
        ctx.step(render=False)
        if (i + 1) % 60 == 0: log(f"  passo {i+1}/240 — Y = {cube_y():.3f}")
    y1 = cube_y(); ctx.stop()
    print("\n===== RESULTADO =====")
    print(f"  Y inicial {y0:.3f} | Y final {y1:.3f} | queda {y0 - y1:.3f}")
    print("  ✅ física OK" if y0 - y1 > 0.05 else "  ⚠️ cubo não caiu — ver colisor/gravidade")
    print("=====================\n")


if __name__ == "__main__":
    try: main()
    finally: sim.close(); sys.exit(0)
