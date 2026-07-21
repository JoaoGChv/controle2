"""
Caminho A — demo no Isaac Sim (5.0/5.1): confirma o 3DGS fotorrealista (NuRec) sobreposto
ao mesh de colisão OCULTO. Larga um cubo dinâmico que cai e COLIDE com a geometria real
da cena (o mesh proxy), enquanto o que se vê é o render fotorrealista dos gaussianos.

USDZ combinado (feito no 4090 com 3DGRUT):
  /World/gauss  -> Volume NuRec (render fotorrealista)
  /World/mesh   -> mesh TSDF, colisão ativada, é o `proxy` do volume (mesmo frame métrico)

Uso (dentro do container, headless):
  ./python.sh /root/dtvf_isaac/run_gs_collision_demo.py d435i_3pass_gs_collision.usdz
Opções: --steps N (passos de física, def 180) --drop 0.6 (altura de largada, m)
Saída: /root/dtvf_isaac/gs_collision/rgb_0000.png (antes) e rgb_0001.png (depois).
"""
import sys
import numpy as np

from isaacsim import SimulationApp
sim = SimulationApp({"headless": True, "width": 1280, "height": 720})

import omni.usd
import omni.replicator.core as rep
from pxr import UsdGeom, UsdLux, UsdPhysics, PhysxSchema, Gf, Sdf, UsdShade

args = sys.argv[1:]
fname = next((a for a in args if a.endswith((".usd", ".usdz", ".usda"))), "d435i_3pass_gs_collision.usdz")
USD = "/root/dtvf_isaac/" + fname
STEPS = int(args[args.index("--steps") + 1]) if "--steps" in args else 180
DROP = float(args[args.index("--drop") + 1]) if "--drop" in args else 0.6
OUT = "/root/dtvf_isaac/gs_collision"

try:
    from isaacsim.core.utils.stage import open_stage
    from isaacsim.core.api import SimulationContext
except Exception:
    from omni.isaac.core.utils.stage import open_stage
    from omni.isaac.core import SimulationContext


def mesh_bounds(stage, path):
    m = UsdGeom.Mesh(stage.GetPrimAtPath(path))
    pts = m.GetPointsAttr().Get()
    if not pts:
        return None
    a = np.array(pts)
    return a.mean(0), a.min(0), a.max(0)


def main():
    open_stage(USD)
    stage = omni.usd.get_context().get_stage()

    # localizar o mesh de colisão (proxy do volume NuRec)
    mesh_path = None
    for p in stage.Traverse():
        if p.GetTypeName() == "Mesh":
            mesh_path = str(p.GetPath()); break
    if mesh_path is None:
        print("[!] nenhum Mesh encontrado no USDZ"); print("GS COLLISION DONE"); return
    print(f"[demo] mesh de colisão: {mesh_path}", flush=True)
    center, mn, mx = mesh_bounds(stage, mesh_path)

    # garantir colisão no mesh (triângulos -> collider estático)
    mprim = stage.GetPrimAtPath(mesh_path)
    UsdPhysics.CollisionAPI.Apply(mprim)
    mca = UsdPhysics.MeshCollisionAPI.Apply(mprim)
    mca.CreateApproximationAttr().Set("none")   # malha exata (estática)

    # cena de física (upAxis=Z -> gravidade em -Z)
    scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/World/physicsScene"))
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    scene.CreateGravityMagnitudeAttr(9.81)

    # ponto de largada: acima do centro da cena, cai até bater na superfície abaixo
    drop_xyz = Gf.Vec3f(float(center[0]), float(center[1]), float(mx[2]) + DROP)
    size = float(min(mx - mn)) * 0.06 + 0.05     # cubo ~ escala da cena
    cube = UsdGeom.Cube.Define(stage, Sdf.Path("/World/DropCube"))
    cube.CreateSizeAttr(size)
    cx = cube.AddTranslateOp(); cx.Set(drop_xyz)
    cp = cube.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(cp)
    UsdPhysics.CollisionAPI.Apply(cp)
    UsdPhysics.MassAPI.Apply(cp).CreateMassAttr(1.0)
    # cor viva para destacar no render
    UsdGeom.Gprim(cp).CreateDisplayColorAttr().Set([(1.0, 0.25, 0.05)])
    print(f"[demo] cubo largado em z={drop_xyz[2]:.2f} (tamanho {size:.2f} m)", flush=True)

    # câmara diagonal a olhar para o centro
    UsdLux.DomeLight.Define(stage, "/World/DemoLight").CreateIntensityAttr(1000.0)
    ext = float(np.linalg.norm(mx - mn))
    eye = center + np.array([ext * 0.28, ext * 0.22, ext * 0.28])
    cam = rep.create.camera()
    rp = rep.create.render_product(cam, (1280, 720))
    writer = rep.WriterRegistry.get("BasicWriter")
    writer.initialize(output_dir=OUT, rgb=True)
    writer.attach([rp])
    with cam:
        rep.modify.pose(position=tuple(map(float, eye)), look_at=tuple(map(float, center)))

    # ANTES (cubo no ar) + simulação
    rep.orchestrator.step(rt_subframes=16)
    sctx = SimulationContext(stage_units_in_meters=1.0)
    sctx.initialize_physics(); sctx.play()
    z0 = drop_xyz[2]
    for i in range(STEPS):
        sctx.step(render=False)
    zf = float(cube.GetPrim().GetAttribute("xformOp:translate").Get()[2])
    print(f"[demo] z cubo: inicial={z0:.3f} -> final={zf:.3f}  (Δqueda={z0-zf:.3f} m)", flush=True)
    print(f"[demo] parou acima do fundo do mesh (z_min={mn[2]:.2f})? "
          f"{'SIM (colidiu)' if zf > mn[2] - 0.2 else 'caiu através (verificar colisão)'}", flush=True)
    # DEPOIS (cubo assente)
    rep.orchestrator.step(rt_subframes=16)
    rep.orchestrator.wait_until_complete()
    print(f"[demo] renders em {OUT}/  (rgb_0000=antes, rgb_0001=depois da queda)", flush=True)
    print("GS COLLISION DONE")


if __name__ == "__main__":
    try:
        main()
    finally:
        sim.close(); sys.exit(0)
