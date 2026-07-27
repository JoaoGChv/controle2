# Prompt para o Claude Code na 2060 — cena D435i (RGB-D)

Copia tudo abaixo da linha para o Claude Code de `pmec-desktop01`.

---

Estou em `pmec-desktop01` (Ubuntu 22.04, 2× RTX 2060 SUPER 8GB, i7-2600K, driver 570, CUDA 12.8), com **Isaac Sim 5.0 em Docker** (`nvcr.io/nvidia/isaac-sim:5.0.0`).

Tenho um bundle em `~/tmp_joaoG/isaac_bundle_lab_v4/` com uma cena reconstruída de uma captura RGB-D (RealSense D435i): mesh **TSDF métrico** colidível + gaussianos como pontos + cubo de teste com física. Ficheiros:
- `lab_v4_isaac_sim.usd` — cena a simular (referencia `lab_v4_isaac.usd` + PhysicsScene gravidade −Y + `/World/TestCube`).
- `lab_v4_isaac.usd` — base: `/World/Scene/Environment` (mesh TSDF + colisor estático) + `/World/Scene/GaussianSplats`.
- `lab_v4_isaac.usdz`, `lab_v4_gaussians.ply`, `lab_v4_mesh.ply`, `lab_v4_mesh_preview.png`.
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
  -v ~/tmp_joaoG/isaac_bundle_lab_v4:/root/dtvf_isaac:rw \
  nvcr.io/nvidia/isaac-sim:5.0.0
```

2) Valida a física (deve terminar com "✅ física OK"):
```bash
ls /root/dtvf_isaac/
./python.sh /root/dtvf_isaac/run_check.py lab_v4_isaac_sim.usd
```

3) Renderiza um PNG:
```bash
./python.sh /root/dtvf_isaac/run_render.py lab_v4_isaac_sim.usd
```
Imagem no host em `~/tmp_joaoG/isaac_bundle_lab_v4/render_lab_v4/rgb_*.png`.

## Notas
- O mesh é **TSDF métrico** (metros reais, ~250k triângulos, voxel 2.5cm) — muito melhor que os Poisson-de-splat anteriores, e já decimado (cooking rápido).
- 1º arranque compila shaders (~7 min); com caches, o 2º cai para ~1–2 min. Não corras dois processos Isaac em simultâneo (8GB → OOM).
- Se `check` der Δ=0 mesmo com `updateToUsd`: troca `ctx.initialize_physics(); ctx.play()` por `from isaacsim.core.api import World; world=World(); world.reset()` e lê a pose com `RigidPrim("/World/TestCube").get_world_poses()`.
- Gaussianos veem-se melhor em https://superspl.at/editor (o Isaac mostra-os só como pontos).

Objetivo: `run_check` a dar "✅ física OK" e um `rgb_0000.png` da cena.

---

## Passo 5 — cortar/remover um objeto (Parte 3 final)
A cena `lab_v4_objects_isaac_sim.usd` tem 34 objetos como prims separados em
`/World/Scene/Objects/obj_NN` (mesh + colisor). Testa remover um e ver o ambiente responder:

```bash
# 1) listar os objetos (nome + centro 3D):
./python.sh /root/dtvf_isaac/run_object_cut.py lab_v4_objects_isaac_sim.usd --list

# 2) remover um (render antes/depois):
./python.sh /root/dtvf_isaac/run_object_cut.py lab_v4_objects_isaac_sim.usd --remove obj_07
```
Saída no host: `cut_obj_07/rgb_0000.png` (antes) e `rgb_0001.png` (depois) — o objeto some da cena.
Podes também abrir o `..._objects_isaac_sim.usd` na GUI (via runheadless+WebRTC) e apagar/mover
prims de `/World/Scene/Objects/` à mão, e correr a física.

`objects_preview.png` — os 34 objetos coloridos (ambiente=cinza).

---

## ⭐ Caminho A — 3DGS fotorrealista (NuRec) + mesh de colisão oculto
Igual ao d435i_3pass: `lab_v4_gs_collision.usdz` = /World/gauss (Volume NuRec) + /World/mesh
(colisão ativada, proxy do volume), co-registados no mesmo frame métrico (Sim(3) exata por
centros de câmara; ver gs_mesh_align_overlay.png). `lab_v4_gs.usdz` = só o GS.

- Abrir na GUI: arrastar `lab_v4_gs_collision.usdz` para o viewport (renderer RTX - Real-Time).
- Teste de colisão headless (larga cubo que pára na superfície):
```bash
./python.sh /root/dtvf_isaac/run_gs_collision_demo.py lab_v4_gs_collision.usdz
```
Renders em `gs_collision/rgb_0000.png` (antes) e `rgb_0001.png` (depois).

---

## ⭐⭐ v2 — splat RETREINADO com 3DGRUT MCMC (qualidade nova geração)
O GS antigo (splatfacto de maio) tinha névoa/floaters/streaks — inviável p/ navegação.
Retreinado no 4090 com 3DGUT+MCMC (regularização de opacidade+escala): PSNR 28.8, SSIM 0.937,
560k gaussianos limpos. Ver `gs_v2_render_sample.png` (render do treino — fotorrealista de verdade).

- **`lab_v4_gs_collision_v2.usdz`** ← USAR ESTE (GS v2 + mesh de colisão, mesmo frame métrico)
- `lab_v4_gs_v2.usdz` — só o GS v2
- Os `lab_v4_gs*.usdz` SEM v2 são a versão antiga (manter só p/ comparação)

Demo de colisão: `./python.sh /root/dtvf_isaac/run_gs_collision_demo.py lab_v4_gs_collision_v2.usdz`

---

## ⭐⭐⭐ ROBÔ A NAVEGAR (digital twin p/ robótica)
Passo novo: um robô com rodas percorre a cena 3DGS fotorrealista colidindo com a geometria real.

**Pré-requisito resolvido no 4090 — gravity align:** o frame COLMAP/TSDF vinha inclinado (o "cima"
não era Z). `gravity_align_scene.py` detetou o chão (RANSAC) e rodou GS+mesh para Z-up com o chão
em z=0. Sem isto o robô escorregaria. Resultado: `lab_v4_robot_scene.usdz` (GS + colisão, Z-up).

Ficheiros:
- `lab_v4_robot_scene.usdz` — cena robot-ready (chão horizontal em z=0).
- `nav_path.json` — start/goal + 13 waypoints (caminho A* de 7.2m na planta, desvia da mobília).
- `nav_path.png` / `lab_v4_occupancy.png` — planta 2D (navegável vs obstáculos).
- `run_robot_nav.py` — carrega a cena, põe Nova Carter no start, segue os waypoints (pure-pursuit),
  grava frames.

Correr:
```bash
./python.sh /root/dtvf_isaac/run_robot_nav.py lab_v4_robot_scene.usdz
# opções: --robot carter|jetbot  --cam follow|top  --steps 3000
```
Saída: `robot_nav/rgb_XXXX.png` (sequência → vídeo do robô a andar no lab fotorrealista).

NOTAS (script pode precisar de ajuste na 2060, não testável no 4090):
- O path do asset do robô varia por versão do Isaac — o script tenta vários; se falhar, ajusta
  `CANDS['carter']` (procura em `get_assets_root_path()/Isaac/Robots/...`).
- Os nomes dos DOF das rodas do Nova Carter podem diferir (`joint_wheel_left/right`); se o robô não
  andar, lista os joints (`robot.dof_names`) e corrige `wheel_dof_names`.
- Há um GroundPlane de segurança em z=0 (evita cair em buracos do mesh). O mesh TSDF também colide.
- Se o robô tombar: baixa a velocidade `v` ou sobe o spawn `z=0.15`.

---

## ⭐⭐⭐⭐ TELEOP ROS2 (PS4 + RViz + lidar) — em construção
Objetivo do user: conduzir o robô com comando PS4, ver no RViz, com lidar, via ROS2 Humble,
com Isaac a manter o render 3DGS. (Gazebo NÃO — não renderiza gaussianos.)

Arquitetura: [PS4]->joy->teleop_twist_joy->/cmd_vel -> [Isaac Nova Carter] -> /scan+TF+/odom -> [RViz2]

MILESTONE 1 — comando PS4 no ROS2 (independente):
  sudo apt install ros-humble-joy ros-humble-teleop-twist-joy
  ls /dev/input/js*            # deve aparecer js0 (USB) ou após parear Bluetooth
  ros2 run joy joy_node        # noutro terminal: ros2 topic echo /joy (mexe os sticks)

MILESTONE 2 — Isaac conduz por /cmd_vel (script run_isaac_teleop.py, reusa o drive validado):
  Terminal A:  source /opt/ros/humble/setup.bash
               ros2 launch teleop_twist_joy teleop-launch.py joy_config:='ps3'
  Terminal B:  source /opt/ros/humble/setup.bash
               <isaac>/python.sh /caminho/lab_v4/run_isaac_teleop.py
  (segura o botão deadman do comando e mexe o stick -> robô anda na sala GS)
  Se andar lento: --speed 3. O script já força controlo por velocidade nas rodas (stiffness=0).

MILESTONE 3 (a seguir) — RTX lidar -> /scan + TF, ver no RViz2 (add script/graph do lidar).
MILESTONE 4 (opcional) — slam_toolbox p/ mapa.

NOTA drive: na nav autónoma o Carter andava lento (rodas em modo posição). run_isaac_teleop.py
põe kps=0/kds=1e5 nas rodas (modo velocidade) — deve resolver.

### FIX teleop (27/07): Isaac py3.11 vs Humble rclpy py3.10 = incompatível
Não dá para importar rclpy no Python do Isaac. Solução: ponte UDP.
- cmd_vel_udp_bridge.py corre no python3 do SISTEMA (ROS sourced) e reenvia /cmd_vel -> UDP 9091.
- run_isaac_teleop.py (Isaac py3.11, SEM source) lê o UDP e conduz. 3 terminais:
  A) source /opt/ros/humble/setup.bash; ros2 launch teleop_twist_joy teleop-launch.py joy_config:='ps3'
  B) source /opt/ros/humble/setup.bash; python3 <lab_v4>/cmd_vel_udp_bridge.py
  C) ~/isaacsim/python.sh <lab_v4>/run_isaac_teleop.py    (NÃO sourced)
Deadman DS4 = botão 8 (SHARE); stick esq. vertical=frente (axis1), horizontal=vira (axis0). --speed 3 se lento.

---

## MESH LIMPA + MILESTONE 3 (lidar + RViz)
Mesh de colisão limpa (clean_collision_mesh.py: removeu 565 fragmentos soltos, tapou buracos
pequenos) -> lab_v4_robot_scene.usdz regenerado. Recopiar esse usdz.

### M3 — lidar 2D -> /scan + TF -> RViz2
Lidar 2D por raycasting dentro do Isaac (evita RTX lidar + OmniGraph; mesma estratégia UDP).
4 terminais:
  A) source /opt/ros/humble/setup.bash
     ros2 run joy joy_node
  A2) source /opt/ros/humble/setup.bash
     ros2 run teleop_twist_joy teleop_node --ros-args -p require_enable_button:=false \
       -p axis_linear.x:=1 -p scale_linear.x:=0.7 -p axis_angular.yaw:=0 -p scale_angular.yaw:=1.0
  B) source /opt/ros/humble/setup.bash
     python3 <lab_v4>/cmd_vel_udp_bridge.py           # PS4 -> /cmd_vel -> UDP 9091
  C) source /opt/ros/humble/setup.bash
     python3 <lab_v4>/isaac_scan_bridge.py            # UDP 9092 -> /scan + TF
  D) ~/isaacsim/python.sh <lab_v4>/run_isaac_teleop_lidar.py   # (SEM source) drive + lidar
  E) source /opt/ros/humble/setup.bash
     rviz2 -d <lab_v4>/lab_v4.rviz                    # fixed frame = odom, mostra /scan + TF

Conduz com o PS4: o robô anda na sala GS (Isaac) e o scan do lidar aparece no RViz2 a varrer as
paredes/mobília. Fluxo: cmd_vel UDP 9091 (entra) ; scan+pose UDP 9092 (sai).
Opções lidar: --rays 180 --lidar-hz 10 --range 12.  TF: map->odom->base_link->laser.
