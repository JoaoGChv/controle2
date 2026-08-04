# Comandos — Robô teleoperado + Lidar + RViz (lab_v4)

Referência única e limpa para correr o digital twin com o robô na máquina do Isaac (2060, nativo).
(A `HANDOFF_CLAUDE_2060.md` tem o histórico detalhado de cada milestone; este é o "cola e corre".)

> **Contexto:** Isaac Sim 5.1 **nativo** + ROS2 **Humble**. Como o Python do Isaac (3.11) é
> incompatível com o `rclpy` do Humble (3.10), a ligação faz-se por **pontes UDP**. Por isso o
> terminal do Isaac corre **SEM** `source` do ROS; os terminais ROS correm **COM** `source`.

---

## 0. Pré-requisito — ficheiros na pasta (2060)
Copiar para `~/tmp_joaoG/controle2/lab_v4/` (ajusta ao teu caminho):
- `lab_v4_robot_scene.usdz`  (cena GS + colisão, gravity-aligned)
- `nav_path.json`            (spawn do robô)
- `run_isaac_teleop_lidar.py` (drive + lidar)  — ou `run_isaac_teleop.py` (só drive)
- `cmd_vel_udp_bridge.py`, `isaac_scan_bridge.py`, `lab_v4.rviz`

Variáveis usadas abaixo:
```bash
BUNDLE=/home/flyingrobots/tmp_joaoG/controle2/lab_v4
ISAAC=~/isaacsim        # pasta de instalação do Isaac Sim (tem o python.sh)
```

---

## 1. Setup completo — teleop + LIDAR + RViz (5 terminais)

```bash
# ── Terminal A ── PS4 → /cmd_vel  (ROS sourced; sem deadman)
source /opt/ros/humble/setup.bash
ros2 run joy joy_node &
ros2 run teleop_twist_joy teleop_node --ros-args -p require_enable_button:=false \
  -p axis_linear.x:=1 -p scale_linear.x:=0.7 -p axis_angular.yaw:=0 -p scale_angular.yaw:=1.0

# ── Terminal B ── ponte cmd_vel → UDP 9091  (ROS sourced)
source /opt/ros/humble/setup.bash
python3 $BUNDLE/cmd_vel_udp_bridge.py

# ── Terminal C ── ponte scan UDP 9092 → /scan + TF (+ /points em 3D)  (ROS sourced)
source /opt/ros/humble/setup.bash
python3 $BUNDLE/isaac_scan_bridge.py

# ── Terminal D ── Isaac: drive + lidar   (NÃO fazer source do ROS!)
$ISAAC/python.sh $BUNDLE/run_isaac_teleop_lidar.py --rays 360

# ── Terminal E ── RViz2  (fixed frame = odom)
source /opt/ros/humble/setup.bash
rviz2 -d $BUNDLE/lab_v4.rviz
```

**Conduzir:** stick esquerdo do PS4 (cima/baixo = frente/trás; esquerda/direita = virar).
No Isaac vês o robô na sala fotorrealista; no RViz vês o `/scan` (e `/points` em 3D).

---

## 2. Variantes

### Só teleop (sem lidar/RViz) — 3 terminais
Usa `run_isaac_teleop.py` (em vez do `_lidar`) e dispensa os terminais **C** e **E**:
```bash
# Terminais A e B iguais; depois:
$ISAAC/python.sh $BUNDLE/run_isaac_teleop.py
```

### Lidar 2D denso (recomendado p/ SLAM 2D)
```bash
$ISAAC/python.sh $BUNDLE/run_isaac_teleop_lidar.py --rays 360
```

### Lidar 3D (nuvem de pontos /points)
```bash
# começar modesto (raycasting é em Python/CPU) e afinar
$ISAAC/python.sh $BUNDLE/run_isaac_teleop_lidar.py --rings 16 --rays 180 --vfov 30 --lidar-hz 5
```
Opções: `--rings` (camadas verticais; 1=2D), `--vfov` (abertura vertical em graus),
`--rays` (azimute), `--lidar-hz`, `--range`, `--speed` (multiplicador de velocidade).

---

## 3. Verificar / depurar
```bash
# tópicos e conteúdo
source /opt/ros/humble/setup.bash
ros2 topic list
ros2 topic echo /cmd_vel --once      # PS4 a publicar? (mexe o stick)
ros2 topic echo /scan --once         # ranges variados (não tudo 12.0)?
ros2 topic hz /points                # taxa da nuvem 3D

# no Terminal D (Isaac) confere as linhas [lidar]:
#   [lidar] rodas: [...]              -> rodas detetadas
#   [lidar] a ouvir UDP ...
#   [lidar] scan: Nanel×Maz hits=.../... dist min/méd/max
```

**Se o robô não anda:** ver Terminal D — comando `cmd=[v.., w..]` a chegar? Se `v=0` sempre,
o problema é a montante (PS4/ponte B). Se anda lento, usa `--speed 3`.

**Se o RViz não mostra o scan:** confirma o `fixed frame = odom` e que o Terminal D imprime
`hits` > 0. Pontos no alcance máximo não são desenhados (é normal).

---

## 4. Gravar vídeo
- Ecrã: OBS ou gravador do GNOME durante a teleoperação.
- (Se usares um writer de frames) sequência → vídeo:
  ```bash
  ffmpeg -framerate 15 -i rgb_%04d.png -c:v libx264 -pix_fmt yuv420p nav.mp4
  ```

---

## 5. Portas / frames (referência)
- UDP **9091**: `/cmd_vel` → Isaac (entra).   UDP **9092**: scan+pose → ROS (sai).
- TF: `map → odom → base_link → laser`.  Tópicos: `/cmd_vel`, `/scan`, `/points` (3D), `/tf`.
