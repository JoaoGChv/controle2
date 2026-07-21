# Prompt para o Claude Code na 2060 — cena D435i (RGB-D)

Copia tudo abaixo da linha para o Claude Code de `pmec-desktop01`.

---

Estou em `pmec-desktop01` (Ubuntu 22.04, 2× RTX 2060 SUPER 8GB, i7-2600K, driver 570, CUDA 12.8), com **Isaac Sim 5.0 em Docker** (`nvcr.io/nvidia/isaac-sim:5.0.0`).

Tenho um bundle em `~/tmp_joaoG/isaac_bundle_d435i_3pass/` com uma cena reconstruída de uma captura RGB-D (RealSense D435i): mesh **TSDF métrico** colidível + gaussianos como pontos + cubo de teste com física. Ficheiros:
- `d435i_3pass_isaac_sim.usd` — cena a simular (referencia `d435i_3pass_isaac.usd` + PhysicsScene gravidade −Y + `/World/TestCube`).
- `d435i_3pass_isaac.usd` — base: `/World/Scene/Environment` (mesh TSDF + colisor estático) + `/World/Scene/GaussianSplats`.
- `d435i_3pass_isaac.usdz`, `d435i_3pass_gaussians.ply`, `d435i_3pass_mesh.ply`, `d435i_3pass_mesh_preview.png`.
- `run_check.py`, `run_render.py` (recebem o nome do USD sim como argumento).

## Tarefa

1) Arranca o container (caches todas p/ 2º arranque rápido; bash; bundle montado):
```bash
mkdir -p ~/docker/isaac-sim/{cache/kit,cache/ov,cache/pip,cache/glcache,cache/computecache,logs,data,documents}
docker stop isaac 2>/dev/null
docker run --name isaac --rm -it --gpus all --network host --entrypoint bash \
  -e "ACCEPT_EULA=Y" -e "PRIVACY_CONSENT=Y" \
  -v ~/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
  -v ~/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
  -v ~/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
  -v ~/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
  -v ~/tmp_joaoG/isaac_bundle_d435i_3pass:/root/dtvf_isaac:rw \
  nvcr.io/nvidia/isaac-sim:5.0.0
```

2) Valida a física (deve terminar com "✅ física OK"):
```bash
ls /root/dtvf_isaac/
./python.sh /root/dtvf_isaac/run_check.py d435i_3pass_isaac_sim.usd
```

3) Renderiza um PNG:
```bash
./python.sh /root/dtvf_isaac/run_render.py d435i_3pass_isaac_sim.usd
```
Imagem no host em `~/tmp_joaoG/isaac_bundle_d435i_3pass/render_d435i_3pass/rgb_*.png`.

## Notas
- O mesh é **TSDF métrico** (metros reais, ~250k triângulos, voxel 2.5cm) — muito melhor que os Poisson-de-splat anteriores, e já decimado (cooking rápido).
- 1º arranque compila shaders (~7 min); com caches, o 2º cai para ~1–2 min. Não corras dois processos Isaac em simultâneo (8GB → OOM).
- Se `check` der Δ=0 mesmo com `updateToUsd`: troca `ctx.initialize_physics(); ctx.play()` por `from isaacsim.core.api import World; world=World(); world.reset()` e lê a pose com `RigidPrim("/World/TestCube").get_world_poses()`.
- Gaussianos veem-se melhor em https://superspl.at/editor (o Isaac mostra-os só como pontos).

Objetivo: `run_check` a dar "✅ física OK" e um `rgb_0000.png` da cena.

---

## Passo 5 — cortar/remover um objeto (Parte 3 final)
A cena `d435i_3pass_objects_isaac_sim.usd` tem 34 objetos como prims separados em
`/World/Scene/Objects/obj_NN` (mesh + colisor). Testa remover um e ver o ambiente responder:

```bash
# 1) listar os objetos (nome + centro 3D):
./python.sh /root/dtvf_isaac/run_object_cut.py d435i_3pass_objects_isaac_sim.usd --list

# 2) remover um (render antes/depois):
./python.sh /root/dtvf_isaac/run_object_cut.py d435i_3pass_objects_isaac_sim.usd --remove obj_07
```
Saída no host: `cut_obj_07/rgb_0000.png` (antes) e `rgb_0001.png` (depois) — o objeto some da cena.
Podes também abrir o `..._objects_isaac_sim.usd` na GUI (via runheadless+WebRTC) e apagar/mover
prims de `/World/Scene/Objects/` à mão, e correr a física.

`objects_preview.png` — os 34 objetos coloridos (ambiente=cinza).

---

## ⭐ Caminho A — 3DGS FOTORREALISTA + mesh de colisão OCULTO (juntos, tipo Omniverse)
Isto substitui os "gaussianos como pontos": agora o GS renderiza a sério (NuRec/RTX) e o mesh
TSDF fica invisível, só para colisão/física. **Precisa de Isaac Sim 5.0–6.0** (o NuRec é do
renderer RTX; renderiza no 5.0). Feito no 4090 com o 3DGRUT.

Ficheiros novos no bundle:
- `d435i_3pass_gs_collision.usdz` — **o entregável**: `/World/gauss` (Volume NuRec, render
  fotorrealista) + `/World/mesh` (mesh TSDF, colisão ativada, é o `proxy` do volume). GS e mesh
  já **co-registados no mesmo frame métrico** (alinhamento Sim(3) exato por centros de câmara +
  filtro de floaters; ver `gs_mesh_align_overlay.png`).
- `d435i_3pass_gs.usdz` — só o GS NuRec (sem colisão), para inspeção visual rápida.
- `run_gs_collision_demo.py` — larga um cubo que cai e colide com a geometria real da cena.

### Ver o render fotorrealista (GUI, AnyDesk)
Abre `d435i_3pass_gs_collision.usdz` no Isaac (arrasta para o viewport, ou File>Open). Põe o
renderer em **RTX - Real-Time** (ou Interactive/Path-Traced). Deves ver a sala fotorrealista
(não pontos). O mesh de colisão está lá mas podes deixá-lo visível ou ocultá-lo (`/World/mesh`).

### Testar a colisão (headless, prova que GS e mesh estão sobrepostos)
```bash
./python.sh /root/dtvf_isaac/run_gs_collision_demo.py d435i_3pass_gs_collision.usdz
# opções: --steps 240 (mais tempo de queda) --drop 0.8 (largar mais alto)
```
Saída no host: `gs_collision/rgb_0000.png` (cubo no ar, sobre o render GS) e `rgb_0001.png`
(cubo assente na superfície). O log diz o Δqueda e se "SIM (colidiu)". O ponto é: o cubo pára
onde a superfície APARECE no render GS → GS e colisão coincidem.

### Notas Caminho A
- `metersPerUnit=1`, `upAxis=Z`, gravidade em −Z. `/World/gauss` tem `xformOp:transform`
  identidade e o mesh é referenciado nas coords nativas → estão no mesmo sítio, não mexas nos xforms.
- Se o NuRec **não** renderizar no 5.0 (aparecer vazio/preto no viewport): confirma o renderer RTX
  (não "Storm"), e testa primeiro o `d435i_3pass_gs.usdz`. Se mesmo assim falhar, dá upgrade p/
  Isaac 5.1/6.0 (o mesmo USDZ serve) — o alinhamento e a física não mudam.
- O mesh usa `MeshCollisionAPI approximation="none"` (malha exata, estática). Se o cooking for
  lento na 2060, troca para `"convexDecomposition"` no script (menos exato, mais rápido).
