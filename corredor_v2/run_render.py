"""Render headless -> PNG (ver sem streaming). Isaac Sim 5.0.
Uso:  ./python.sh /root/dtvf_isaac/run_render.py <cena>_isaac_sim.usd
Saída: /root/dtvf_isaac/render_<cena>/rgb_*.png
"""
import sys
from isaacsim import SimulationApp
sim = SimulationApp({"headless": True, "width": 1280, "height": 720})

import numpy as np, omni.usd, omni.replicator.core as rep
from pxr import UsdGeom, UsdLux

fname = sys.argv[1] if len(sys.argv) > 1 else "video1_isaac_sim.usd"
USD = "/root/dtvf_isaac/" + fname
OUT = "/root/dtvf_isaac/render_" + fname.replace("_isaac_sim.usd", "")

try:
    from isaacsim.core.api import SimulationContext
    from isaacsim.core.utils.stage import open_stage
except Exception:
    from omni.isaac.core import SimulationContext
    from omni.isaac.core.utils.stage import open_stage


def main():
    open_stage(USD); stage = omni.usd.get_context().get_stage()
    UsdLux.DomeLight.Define(stage, "/World/Lights/Dome").CreateIntensityAttr(1000.0)
    pts = np.array(UsdGeom.Mesh(stage.GetPrimAtPath("/World/Scene/Environment")).GetPointsAttr().Get())
    cmin, cmax = pts.min(0), pts.max(0); center = (cmin + cmax) / 2; ext = float(np.linalg.norm(cmax - cmin))
    eye = center + np.array([ext * 0.6, ext * 0.5, ext * 0.6])
    cam = rep.create.camera(position=tuple(map(float, eye)), look_at=tuple(map(float, center)))
    rp = rep.create.render_product(cam, (1280, 720))
    writer = rep.WriterRegistry.get("BasicWriter"); writer.initialize(output_dir=OUT, rgb=True); writer.attach([rp])
    import carb; carb.settings.get_settings().set_bool("/physics/updateToUsd", True)
    ctx = SimulationContext(stage_units_in_meters=1.0); ctx.initialize_physics(); ctx.play()
    for _ in range(120): ctx.step(render=False)
    for _ in range(6): rep.orchestrator.step()
    print(f"\n✅ imagens em {OUT}/  (rgb_*.png)", flush=True)


if __name__ == "__main__":
    try: main()
    finally: sim.close(); sys.exit(0)
