"""
Limpa uma mesh TSDF para colisão de robô: remove fragmentos soltos (toques fantasma),
tapa buracos pequenos e limpa geometria degenerada. Não move vértices -> continua alinhada
com o GS. O chão global fica garantido por um GroundPlane no Isaac; aqui tratamos das paredes,
mobília e fragmentos.

Uso: python clean_collision_mesh.py <in.ply> <out.ply> [--min-frac 0.005] [--hole 0.15]
  --min-frac: remove componentes com < frac do maior componente (fragmentos)
  --hole: tamanho máx (m) de buraco a tapar (0 = não tapar)
Env: difix3d_ns (open3d).
"""
import argparse
import numpy as np
import open3d as o3d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("inp"); ap.add_argument("out")
    ap.add_argument("--min-frac", type=float, default=0.005)
    ap.add_argument("--hole", type=float, default=0.15)
    a = ap.parse_args()

    m = o3d.io.read_triangle_mesh(a.inp)
    n0 = len(m.triangles)
    print(f"entrada: {len(m.vertices)} verts, {n0} tri", flush=True)

    # limpeza básica
    m.remove_duplicated_vertices()
    m.remove_duplicated_triangles()
    m.remove_degenerate_triangles()
    m.remove_non_manifold_edges()

    # remover fragmentos: mantém componentes >= min_frac do maior
    idx, counts, _ = m.cluster_connected_triangles()
    idx = np.asarray(idx); counts = np.asarray(counts)
    keep = set(np.where(counts >= counts.max() * a.min_frac)[0].tolist())
    rm = np.where(~np.isin(idx, list(keep)))[0]
    m.remove_triangles_by_index(rm.tolist())
    m.remove_unreferenced_vertices()
    print(f"fragmentos: {len(counts)} componentes -> mantidos {len(keep)} "
          f"(removidos {len(rm)} tri)", flush=True)

    # tapar buracos pequenos (tensor API)
    if a.hole > 0:
        try:
            tm = o3d.t.geometry.TriangleMesh.from_legacy(m)
            tm = tm.fill_holes(hole_size=a.hole)
            m = tm.to_legacy()
            print(f"buracos <= {a.hole}m tapados", flush=True)
        except Exception as e:
            print(f"aviso: fill_holes falhou ({e}) — segue sem tapar", flush=True)

    m.remove_degenerate_triangles(); m.remove_unreferenced_vertices()
    m.compute_vertex_normals()
    o3d.io.write_triangle_mesh(a.out, m)
    print(f"-> {a.out}: {len(m.vertices)} verts, {len(m.triangles)} tri "
          f"({100*len(m.triangles)/max(n0,1):.0f}% do original)", flush=True)
    print("CLEAN DONE")


if __name__ == "__main__":
    main()
