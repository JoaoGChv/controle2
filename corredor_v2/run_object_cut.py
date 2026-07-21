"""
Passo 5 — interação no Isaac Sim 5.0: listar objetos e REMOVER/CORTAR um, com render
antes/depois para ver o ambiente responder. Objetos estão em /World/Scene/Objects/obj_NN.

Uso (dentro do container):
  ./python.sh /root/dtvf_isaac/run_object_cut.py <cena>_objects_isaac_sim.usd --list
  ./python.sh /root/dtvf_isaac/run_object_cut.py <cena>_objects_isaac_sim.usd --remove obj_07
Saída: /root/dtvf_isaac/cut_<obj>/render_before.png e render_after.png
"""
import sys
import numpy as np

from isaacsim import SimulationApp
sim = SimulationApp({"headless": True, "width": 1280, "height": 720})

import omni.usd
import omni.replicator.core as rep
from pxr import UsdGeom, UsdLux, Gf

args = sys.argv[1:]
fname = args[0] if args else "d435i_3pass_objects_isaac_sim.usd"
USD = "/root/dtvf_isaac/" + fname
LIST = "--list" in args
REMOVE = args[args.index("--remove") + 1] if "--remove" in args else None

try:
    from isaacsim.core.utils.stage import open_stage
except Exception:
    from omni.isaac.core.utils.stage import open_stage

OBJROOT = "/World/Scene/Objects"


def prim_center(prim):
    pts = UsdGeom.Mesh(prim).GetPointsAttr().Get()
    if not pts:
        return None
    a = np.array(pts)
    return a.mean(0), a.min(0), a.max(0)


def main():
    open_stage(USD)
    stage = omni.usd.get_context().get_stage()
    objs = [p for p in stage.Traverse() if str(p.GetPath()).startswith(OBJROOT + "/obj_")]

    if LIST or not REMOVE:
        print(f"\n=== {len(objs)} objetos em {OBJROOT} ===", flush=True)
        for p in objs:
            c = prim_center(p)
            ctr = np.round(c[0], 2) if c else "?"
            print(f"  {p.GetName()}  centro={ctr}", flush=True)
        print("\nRemove com: --remove <nome>  (ex.: --remove " + (objs[0].GetName() if objs else "obj_00") + ")", flush=True)
        print("OBJECT LIST DONE")
        return

    target = stage.GetPrimAtPath(f"{OBJROOT}/{REMOVE}")
    if not target or not target.IsValid():
        print(f"[!] objeto {REMOVE} não encontrado"); print("OBJECT CUT DONE"); return
    c, cmin, cmax = prim_center(target)

    # câmara a olhar para o objeto
    UsdLux.DomeLight.Define(stage, "/World/CutLight").CreateIntensityAttr(1200.0)
    # ponto de vista: recua ao longo da diagonal da cena a partir do objeto
    envpts = np.array(UsdGeom.Mesh(stage.GetPrimAtPath("/World/Scene/Environment")).GetPointsAttr().Get())
    scene_ext = float(np.linalg.norm(envpts.max(0) - envpts.min(0)))
    eye = c + np.array([scene_ext * 0.25, scene_ext * 0.2, scene_ext * 0.25])
    OUT = f"/root/dtvf_isaac/cut_{REMOVE}"
    cam = rep.create.camera()
    rp = rep.create.render_product(cam, (1280, 720))
    writer = rep.WriterRegistry.get("BasicWriter")
    writer.initialize(output_dir=OUT, rgb=True)
    writer.attach([rp])
    with cam:
        rep.modify.pose(position=tuple(map(float, eye)), look_at=tuple(map(float, c)))

    # ANTES
    rep.orchestrator.step(rt_subframes=16)
    # REMOVE o objeto (desativa -> some da cena e da física)
    target.SetActive(False)
    print(f"[cut] removido {REMOVE} (centro {np.round(c,2)})", flush=True)
    # DEPOIS
    rep.orchestrator.step(rt_subframes=16)
    rep.orchestrator.wait_until_complete()
    print(f"[cut] renders em {OUT}/  (rgb_0000=antes, rgb_0001=depois)", flush=True)
    print("OBJECT CUT DONE")


if __name__ == "__main__":
    try:
        main()
    finally:
        sim.close(); sys.exit(0)
