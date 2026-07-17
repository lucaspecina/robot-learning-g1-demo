# Ecosistema Unitree: comunicación, datos y simulación (Go2 / G1)

> Estado: julio 2026. Objetivo: entender el formato de la data, topics, comandos — primero contra el robot simulado, después el real.

## 1. Cómo habla el Go2 real

- **Middleware nativo: DDS (CycloneDDS)**. No hay servidor propietario: el robot publica y suscribe topics DDS directo en la red. Como ROS 2 también usa DDS, los topics se ven desde ROS 2 con `rmw_cyclonedds_cpp` sin driver.
- **Conexión física**: ethernet al RJ45 del robot. Mi PC en `192.168.123.99/24`; el robot es `192.168.123.161`. Al inicializar el SDK: `ChannelFactoryInitialize(0, "nombre_interfaz")` — el `0` es el DDS domain id (0 = robot real, 1 = sim por convención).
- **Secondary development (DDS abierto) es oficial solo en la versión EDU** (la mía ✓). En Air/Pro solo vía jailbreak, que Unitree bloquea desde firmware 1.1.1.

### Topics principales (Go2)
| Topic | Dirección | Qué es |
|---|---|---|
| `rt/lowstate` | robot → yo | Estado bajo nivel a ~500 Hz: 12 motores (q, dq, tau_est, temp), IMU (quat, gyro, accel, rpy), foot_force[4], batería. También `rt/lf/lowstate` (baja frecuencia). |
| `rt/lowcmd` | yo → robot | Comando de motores. Por motor: `q, dq, tau, kp, kd`. El motor ejecuta `tau_out = tau + kp*(q - q_med) + kd*(dq - dq_med)`. CRC obligatorio. |
| `rt/sportmodestate` | robot → yo | Estado del controlador de alto nivel: posición/velocidad estimada del cuerpo, gait, foot positions. |
| `rt/api/sport/request` / `...response` | request-response | Lo que usa SportClient por debajo. |
| `rt/wirelesscontroller` | robot → yo | Joystick del fabricante. |

### Alto nivel vs bajo nivel
- **Alto nivel (`SportClient`)**: request-response contra el servicio de movimiento que corre A BORDO del robot ("sport mode" / servicio `mcf` en firmwares nuevos). Métodos: `Damp()`, `StandUp()`, `StandDown()`, `Move(vx, vy, vyaw)`, `StopMove()`, `BalanceStand()`, `RecoveryStand()`, `BackFlip()`, `TrajectoryFollow(...)`, etc. Docs: https://support.unitree.com/home/en/developer/sports_services
- **Bajo nivel**: publicar `LowCmd` en `rt/lowcmd` a 500 Hz–1 kHz. **Antes hay que liberar el servicio de movimiento a bordo** con `MotionSwitcherClient.CheckMode()/ReleaseMode()` en loop (si no, el sport mode y mi comando pelean por los motores). Con el sport service apagado, `rt/sportmodestate` deja de publicarse en el robot real.
- **G1**: mismo patrón pero otra familia de mensajes — IDL `unitree_hg` (35 motores, `mode_pr`, `mode_machine`) en vez de `unitree_go` (Go2/B2/H1). El cliente de alto nivel del G1 es `LocoClient` (`Squat2StandUp`, `Move`, `WaveHand`...), no SportClient.

### Campos de `LowState_` (unitree_go)
`imu_state` (quaternion[4], gyroscope[3], accelerometer[3], rpy[3]), `motor_state[20]` (Go2 usa los primeros 12: FR_hip/thigh/calf, FL_…, RR_…, RL_… con q, dq, ddq, tau_est, temperature), `foot_force[4]`, `power_v/power_a` (batería), `tick`, `crc`, etc.

## 2. SDKs

- **unitree_sdk2** (C++): oficial, sobre CycloneDDS. Ejemplos por robot (`go2_stand_example.cpp`, sport client...). https://github.com/unitreerobotics/unitree_sdk2
- **unitree_sdk2_python**: espejo en Python (`pip3 install -e .`; pin `cyclonedds==0.10.2`). Trae `ChannelPublisher/Subscriber`, `SportClient`, `MotionSwitcherClient`, `LocoClient` (G1), IDLs, CRC. Ejemplos: `go2/high_level/go2_sport_client.py`, `go2/low_level/go2_stand_example.py`, carpeta `g1/`. https://github.com/unitreerobotics/unitree_sdk2_python
- **unitree_ros2**: no es un driver; son las definiciones de mensajes ROS 2 + config CycloneDDS para ver los topics nativos (`/lowstate`, `/sportmodestate`...) desde ROS 2 Humble. https://github.com/unitreerobotics/unitree_ros2

## 3. unitree_mujoco — la pieza clave para arrancar

https://github.com/unitreerobotics/unitree_mujoco — simulador oficial sobre MuJoCo. Corre el MJCF del robot + un **bridge que expone por DDS exactamente los mismos topics e IDLs que el robot real**. El código cliente NO cambia entre sim y real — solo `ChannelFactoryInitialize(1, "lo")` (sim) vs `ChannelFactoryInitialize(0, "enpXsY")` (real).

- Robots: go2, g1 (23/29 dof), h1, b2, go2w, a2, h2, r1.
- Publica: `rt/lowstate`, `rt/sportmodestate` (con ground truth del cuerpo — ¡odometría gratis que en el real no tenés en modo low-level!), `rt/wirelesscontroller` (gamepad USB).
- Suscribe: **solo `rt/lowcmd`**.
- ⚠️ **NO soporta alto nivel**: no hay servicio sport mode en el sim → `SportClient.Move()` muere en timeout. *"Current version only supports low-level development."* La API de alto nivel solo se practica contra el robot real (o `unitree_sim_isaaclab` para G1).
- Dos versiones: C++ (recomendada, más rápida) y Python (`simulate_python/`, más simple de tocar). Config: `ROBOT="go2"`, `DOMAIN_ID=1`, `INTERFACE="lo"`, `SIMULATE_DT=0.005`, banda elástica virtual para colgar G1/H1.
- Incluye `terrain_tool` y ejemplo `stand_go2` en cpp/python/ros2.

### Sim vs real — qué se puede
| Capacidad | Go2 real (EDU) | unitree_mujoco |
|---|---|---|
| `rt/lowcmd` (q,dq,tau,kp,kd) | Sí (tras ReleaseMode) | Sí |
| `rt/lowstate` | Sí | Sí (sin batería real) |
| SportClient (Move, StandUp...) | Sí | **NO** |
| `rt/sportmodestate` | Solo con sport service activo | Siempre (ground truth) |
| Cámara / lidar / audio | Sí | No |

## 4. Snippet de referencia (idéntico sim ↔ real)

```python
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_, LowCmd_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.utils.crc import CRC

ChannelFactoryInitialize(1, "lo")   # SIM. Real: (0, "enp2s0")

def on_lowstate(msg: LowState_):
    q0   = msg.motor_state[0].q        # FR_hip [rad]
    quat = msg.imu_state.quaternion    # también .gyroscope, .accelerometer, .rpy
    feet = msg.foot_force              # 4 sensores de pie

sub = ChannelSubscriber("rt/lowstate", LowState_); sub.Init(on_lowstate, 10)

pub = ChannelPublisher("rt/lowcmd", LowCmd_); pub.Init()
cmd = unitree_go_msg_dds__LowCmd_()
cmd.head[0], cmd.head[1], cmd.level_flag = 0xFE, 0xEF, 0xFF
for i in range(20): cmd.motor_cmd[i].mode = 0x01   # FOC
# en loop a 500 Hz:
cmd.motor_cmd[0].q, cmd.motor_cmd[0].kp, cmd.motor_cmd[0].kd = 0.0, 60.0, 5.0
cmd.crc = CRC().Crc(cmd)   # obligatorio
pub.Write(cmd)
```

## 5. Pipeline RL sim-to-real (para después)

Pipeline canónico Unitree: **Train** (miles de envs GPU, PPO con rsl_rl, domain randomization) → **Play** (verificación visual) → **Sim2Sim** (validar en unitree_mujoco) → **Sim2Real** (política a ~50 Hz vía lowcmd, primero con arnés).

- Repos oficiales: `unitree_rl_gym` (legacy, Isaac Gym, trae políticas pre-entrenadas en `deploy/pre_train/`), `unitree_rl_lab` (Isaac Lab, Go2/H1/G1), `unitree_rl_mjlab` (mjlab/MuJoCo Warp, el más nuevo).
- Comunidad: walk-these-ways-go2, HIMLoco, himloco_lab, go2_omniverse. Lista curada: https://github.com/apexrl/awesome-rl-for-legged-locomotion
- Errores comunes: saltear sim2sim, subestimar domain randomization / modelado de actuador (ahí vive el gap sim-to-real), rewards sin regularizar (reward hacking), no apagar el sport_mode antes de lowcmd.

## 6. Curso ETH Zürich

- El curso del RSL (lab de Marco Hutter, creador de ANYmal/legged_gym/rsl_rl) es **"Robot Dynamics" (151-0851-00)**: cinemática/dinámica multi-cuerpo, brazos, legged robots. Ejercicios en MATLAB. https://rsl.ethz.ch/education-students/lectures/robotdynamics.html
- Lectures completas en YouTube: playlist "Robot Dynamics HS 2020": https://www.youtube.com/playlist?list=PLE-BQwvVGf8EfB6I76CSsR7x9hz-r7S6U
- No es un curso de RL: es la base de modelado/control (frames, jacobianos, contactos) — el complemento perfecto de la parte práctica.
