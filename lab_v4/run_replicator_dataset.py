"""
Gerador de dataset sintético (Isaac Sim 5.0 + Replicator) a partir de um USD reconstruído.
Orbita uma câmara à volta da cena e grava, por frame:
  rgb/  depth (distance_to_image_plane)/  segmentação semântica/  + camera_params (intrínsecas+pose).

O teu gémeo digital vira fábrica de dados: gera muito mais vistas rotuladas do que capturaste.

Uso (dentro do container):
  ./python.sh /root/dtvf_isaac/run_replicator_dataset.py <cena>_isaac.usd [n_frames] [radius_factor]
Saída: /root/dtvf_isaac/dataset_<cena>/   (rgb, distance_to_image_plane, semantic_segmentation, ...)
"""
import sys
import numpy as np

from isaacsim import SimulationApp
sim = SimulationApp({"headless": True, "width": 1280, "height": 720})

import omni.usd
import omni.replicator.core as rep
from pxr import UsdGeom

fname = sys.argv[1] if len(sys.argv) > 1 else "d435i_scene_isaac.usd"
N = int(sys.argv[2]) if len(sys.argv) > 2 else 120
radius_factor = float(sys.argv[3]) if len(sys.argv) > 3 else 1.1
USD = "/root/dtvf_isaac/" + fname
OUT = "/root/dtvf_isaac/dataset_" + fname.replace("_isaac.usd", "").replace(".usd", "")

try:
    from isaacsim.core.utils.stage import open_stage
except Exception:
    from omni.isaac.core.utils.stage import open_stage


def add_semantics(prim, label):
    """Rotula um prim com classe semântica (p/ a saída de segmentação)."""
    try:
        from isaacsim.core.utils.semantics import add_update_semantics
    except Exception:
        try:
            from omni.isaac.core.utils.semantics import add_update_semantics
        except Exception:
            return
    add_update_semantics(prim, label)


def main():
    open_stage(USD)
    stage = omni.usd.get_context().get_stage()

    # bbox da cena a partir do mesh do ambiente
    mesh_prim = stage.GetPrimAtPath("/World/Scene/Environment")
    if not mesh_prim or not mesh_prim.IsValid():
        mesh_prim = stage.GetPrimAtPath("/World/Environment")
    pts = np.array(UsdGeom.Mesh(mesh_prim).GetPointsAttr().Get())
    cmin, cmax = pts.min(0), pts.max(0)
    center = (cmin + cmax) / 2
    ext = cmax - cmin
    radius = radius_factor * 0.5 * max(ext[0], ext[2])

    # rótulos semânticos (sem estes, a segmentação sai vazia)
    add_semantics(mesh_prim, "environment")
    gs = stage.GetPrimAtPath("/World/Scene/GaussianSplats")
    if gs and gs.IsValid():
        add_semantics(gs, "points")

    # luz p/ não sair preto
    from pxr import UsdLux
    UsdLux.DomeLight.Define(stage, "/World/RepLight").CreateIntensityAttr(1200.0)

    # poses em órbita (variando azimute e elevação) a olhar p/ o centro
    poses = []
    for i in range(N):
        az = 2 * np.pi * (i / N) * 2.0            # 2 voltas
        el = 0.25 + 0.5 * (i / N)                 # sobe a elevação ao longo da órbita
        pos = center + radius * np.array([np.cos(az) * np.cos(el),
                                          np.sin(el) * (ext[1] / max(ext[0], 1e-3) + 0.5),
                                          np.sin(az) * np.cos(el)])
        poses.append(pos)

    cam = rep.create.camera(focal_length=24.0)
    rp = rep.create.render_product(cam, (1280, 720))
    writer = rep.WriterRegistry.get("BasicWriter")
    writer.initialize(
        output_dir=OUT, rgb=True,
        distance_to_image_plane=True,          # depth
        semantic_segmentation=True,
        instance_id_segmentation=True,
        colorize_semantic_segmentation=True,
        camera_params=True,                    # intrínsecas + pose por frame
    )
    writer.attach([rp])

    print(f"[replicator] {N} frames, centro={np.round(center,2)}, raio={radius:.2f} -> {OUT}", flush=True)
    for i, p in enumerate(poses):
        with cam:
            rep.modify.pose(position=tuple(map(float, p)), look_at=tuple(map(float, center)))
        rep.orchestrator.step(rt_subframes=8)
        if (i + 1) % 20 == 0:
            print(f"  {i+1}/{N}", flush=True)
    rep.orchestrator.wait_until_complete()
    print(f"[replicator] DATASET pronto em {OUT}/", flush=True)
    print("REPLICATOR DONE")


if __name__ == "__main__":
    try:
        main()
    finally:
        sim.close()
        sys.exit(0)
