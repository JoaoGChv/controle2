# Prompt para o Claude Code na 2060 — cena D435i (RGB-D)

Copia tudo abaixo da linha para o Claude Code de `pmec-desktop01`.

---

Estou em `pmec-desktop01` (Ubuntu 22.04, 2× RTX 2060 SUPER 8GB, i7-2600K, driver 570, CUDA 12.8), com **Isaac Sim 5.0 em Docker** (`nvcr.io/nvidia/isaac-sim:5.0.0`).

Tenho um bundle em `~/tmp_joaoG/isaac_bundle_corredor_v2/` com uma cena reconstruída de uma captura RGB-D (RealSense D435i): mesh **TSDF métrico** colidível + gaussianos como pontos + cubo de teste com física. Ficheiros:
- `corredor_v2_isaac_sim.usd` — cena a simular (referencia `corredor_v2_isaac.usd` + PhysicsScene gravidade −Y + `/World/TestCube`).
- `corredor_v2_isaac.usd` — base: `/World/Scene/Environment` (mesh TSDF + colisor estático) + `/World/Scene/GaussianSplats`.
- `corredor_v2_isaac.usdz`, `corredor_v2_gaussians.ply`, `corredor_v2_mesh.ply`, `corredor_v2_mesh_preview.png`.
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
  -v ~/tmp_joaoG/isaac_bundle_corredor_v2:/root/dtvf_isaac:rw \
  nvcr.io/nvidia/isaac-sim:5.0.0
```

2) Valida a física (deve terminar com "✅ física OK"):
```bash
ls /root/dtvf_isaac/
./python.sh /root/dtvf_isaac/run_check.py corredor_v2_isaac_sim.usd
```

3) Renderiza um PNG:
```bash
./python.sh /root/dtvf_isaac/run_render.py corredor_v2_isaac_sim.usd
```
Imagem no host em `~/tmp_joaoG/isaac_bundle_corredor_v2/render_corredor_v2/rgb_*.png`.

## Notas
- O mesh é **TSDF métrico** (metros reais, ~250k triângulos, voxel 2.5cm) — muito melhor que os Poisson-de-splat anteriores, e já decimado (cooking rápido).
- 1º arranque compila shaders (~7 min); com caches, o 2º cai para ~1–2 min. Não corras dois processos Isaac em simultâneo (8GB → OOM).
- Se `check` der Δ=0 mesmo com `updateToUsd`: troca `ctx.initialize_physics(); ctx.play()` por `from isaacsim.core.api import World; world=World(); world.reset()` e lê a pose com `RigidPrim("/World/TestCube").get_world_poses()`.
- Gaussianos veem-se melhor em https://superspl.at/editor (o Isaac mostra-os só como pontos).

Objetivo: `run_check` a dar "✅ física OK" e um `rgb_0000.png` da cena.

---

## Passo 5 — cortar/remover um objeto (Parte 3 final)
A cena `corredor_v2_objects_isaac_sim.usd` tem 34 objetos como prims separados em
`/World/Scene/Objects/obj_NN` (mesh + colisor). Testa remover um e ver o ambiente responder:

```bash
# 1) listar os objetos (nome + centro 3D):
./python.sh /root/dtvf_isaac/run_object_cut.py corredor_v2_objects_isaac_sim.usd --list

# 2) remover um (render antes/depois):
./python.sh /root/dtvf_isaac/run_object_cut.py corredor_v2_objects_isaac_sim.usd --remove obj_07
```
Saída no host: `cut_obj_07/rgb_0000.png` (antes) e `rgb_0001.png` (depois) — o objeto some da cena.
Podes também abrir o `..._objects_isaac_sim.usd` na GUI (via runheadless+WebRTC) e apagar/mover
prims de `/World/Scene/Objects/` à mão, e correr a física.

`objects_preview.png` — os 34 objetos coloridos (ambiente=cinza).
