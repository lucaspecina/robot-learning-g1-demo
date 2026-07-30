# robot-learning-g1-demo

Laboratorio sim → real para robots Unitree. **Objetivo: una demo completa con
el G1 EDU (humanoide)**. Una persona le habla al robot; el robot guarda dónde
empezó, encuentra y lee un reloj, busca la mesa roja o azul que corresponde a
la hora, toma el objeto que haya sobre ella y vuelve al punto de partida.

Principio del proyecto: **todo simulado, pero realista** — no solo la física
del robot, sino también las computadoras donde corre cada cosa (la de a bordo,
el servidor) y las redes entre ellas, con sus latencias, cortes y límites.
El código que se desarrolla acá debe correr en el robot real sin cambios.

## Fases

| Fase | Qué | Estado |
|---|---|---|
| 1 | Laboratorio local: Go2 en MuJoCo, control por DDS crudo, policies RL | hecha (base de aprendizaje; docs abajo) |
| 2 | Laboratorio cloud: Isaac Sim en Azure, ROS 2, sistemas simulados | funcionando |
| 3 | G1: locomoción, navegación, percepción, agente, manipulación, la demo | en integración |

Contexto de fondo: [docs/00_mapa_simuladores.md](docs/00_mapa_simuladores.md),
[docs/01_unitree_ecosistema_datos.md](docs/01_unitree_ecosistema_datos.md),
[docs/02_arquitectura_inteligencia.md](docs/02_arquitectura_inteligencia.md),
[docs/GLOSARIO.md](docs/GLOSARIO.md).

Estado y próximo tramo del G1:
[g1/README.md](g1/README.md),
[g1/DEMO_TARGET.md](g1/DEMO_TARGET.md) y
[g1/AGENT_EXECUTION_PLAN.md](g1/AGENT_EXECUTION_PLAN.md).

## Fase 2: laboratorio cloud (Isaac Sim + ROS 2 + sistemas simulados)

Corre en una VM de Azure con GPU (`rg-go2-lab` / `vm-go2-isaac` — nombres
internos de la época Go2, sin valor semántico). Encendido y apagado:

```bash
bash scripts/lab_up.sh      # VM -> simulador -> jupyter -> túnel (la terminal queda de túnel)
bash scripts/lab_down.sh    # cierra todo y desasigna la VM (deja de facturar)
```

- **El simulador**: fork propio de isaac-go2-ros2 porteado a Isaac Sim 5.1 /
  IsaacLab / ROS 2 Jazzy ([isaac-go2-ros2-port](https://github.com/lucaspecina/isaac-go2-ros2-port),
  rama `port-isaac51`): Go2 con policy de locomoción, driver/bridge ROS 2
  (con watchdog de cmd_vel), cámara a bordo publicando.
- **La notebook** [03_ros2_observatory.ipynb](notebooks/03_ros2_observatory.ipynb):
  hablarle al robot desde un kernel en la VM — censo de topics, odometría,
  comandos, un mini-navegador y la cámara. Su celda 0 es el manual de arranque.
- **Los sistemas simulados** ([systems/](systems/)): la "Jetson" (contenedor
  con ROS 2, 2 CPU / 8 GB, CycloneDDS — ver el porqué en su Dockerfile) y el
  "server" (contenedor en red aparte), con el enlace entre ellos degradable
  a perfiles de wifi reales (`degrade_network.sh`). Curva medida: red limpia
  ~2 ms; wifi normal ~43 ms; wifi malo ~190 ms con picos de 1.3 s por
  retransmisiones TCP; corte 100% pérdida.

## Fase 1: laboratorio local (Go2 en MuJoCo, DDS crudo)

Todo lo que sigue es de la fase 1 — sigue siendo la mejor referencia del nivel
bajo (lowstate/lowcmd, control PD, policies) y funciona igual que siempre.

### Las piezas y qué hace cada una

```
┌─ tu código (scripts/, notebooks/) ──────────────┐
│  usa unitree_sdk2py: publica rt/lowcmd,         │
│  se suscribe a rt/lowstate                      │
└───────────────┬─────────────────────────────────┘
                │ DDS (CycloneDDS) — domain 1, interfaz "lo" (sim)
                │                    domain 0, ethernet     (real)
┌───────────────┴─────────────────────────────────┐
│  SIM:  unitree_mujoco (bridge DDS + física      │
│        MuJoCo + viewer)                         │
│  REAL: el Go2 por el RJ45 (192.168.123.161)     │
└─────────────────────────────────────────────────┘
```

El punto clave: **el código cliente es idéntico en sim y real** — solo cambia
`ChannelFactoryInitialize(1, "lo")` ↔ `(0, "eth0")`. Por eso todos los scripts
aceptan `--real IFACE`.

- **unitree_sdk2_python** (`external/`, instalado editable): el SDK. Trae los
  IDLs (definiciones de mensajes DDS como `LowState_`/`LowCmd_`), pub/sub, CRC,
  y clientes de alto nivel (`SportClient` — solo robot real).
- **unitree_mujoco** (`external/`): simulador oficial. Corre el modelo MJCF del
  Go2 en MuJoCo y expone por DDS los mismos topics que el robot físico.
  Solo soporta **bajo nivel** (`rt/lowcmd`); `SportClient` no funciona en sim.
- **CycloneDDS**: el middleware pub/sub descentralizado (el mismo que usa ROS 2).
  No hay servidor: los procesos se descubren solos en la red/loopback.

### Entorno (WSL2)

Todo corre en **WSL2 Ubuntu 24.04** (el stack Unitree es Linux-only). La GUI de
MuJoCo funciona vía WSLg. GPU RTX 4000 Ada visible desde WSL (para mjlab/RL más
adelante; el sim clásico de MuJoCo es CPU).

- Venv: `~/.venvs/go2` (**Python 3.10**, en el filesystem de WSL por velocidad).
  ¿Por qué 3.10? El SDK pinea `cyclonedds==0.10.2` (2022), que solo tiene wheels
  precompilados hasta cp310; con Python más nuevo habría que compilar CycloneDDS.
- Manejado con [uv](https://docs.astral.sh/uv/). Recrear desde cero:
  ```bash
  uv venv ~/.venvs/go2 --python 3.10
  uv pip install --python ~/.venvs/go2/bin/python \
      -e external/unitree_sdk2_python mujoco pygame matplotlib ipykernel jupyterlab \
      torch pyyaml
  ```
  (o directamente `bash scripts/setup.sh`, que además clona los repos de `external/`)

### Uso (dos terminales WSL)

```bash
source ~/.venvs/go2/bin/activate

# Terminal A: simulador (abre ventana de MuJoCo con el Go2)
bash scripts/sim.sh

# Terminal B: scripts contra el sim
python scripts/00_read_lowstate.py        # ver telemetría en vivo
python scripts/01_stand.py                # pararse y agacharse (control PD)
python scripts/02_record_lowstate.py      # grabar datos a data/*.npz
python scripts/03_walk_policy.py          # CAMINAR: policy RL + teleop (w/s/a/d/q/e, x=salir)
```

#### Policy de locomoción (03_walk_policy.py)

Corre una policy pre-entrenada de [rl_sar](https://github.com/fan-ziqi/rl_sar)
(`external/rl_sar/policy/go2/`): rampa a pose nominal → loop de policy a 50 Hz →
teleop por teclado. Dos disponibles con `--policy`:

- **`himloco`** (default): HIMLoco, obs de 45 con historia de 6 pasos (input 270),
  orden de articulaciones propio (FL,FR,RL,RR — el script traduce). La más limpia:
  gyro std ~0.2 rad/s trotando en nuestro sim.
- **`robot_lab`**: IsaacLab, 45 obs sin historia, orden = SDK. Funciona pero más
  sucia (gyro std ~0.45, deriva de rumbo notoria).

| Tecla | Efecto | | Tecla | Efecto |
|---|---|---|---|---|
| `w` / `s` | vx ±0.1 m/s (adelante/atrás) | | `q` / `e` | vyaw ±0.1 rad/s (girar) |
| `a` / `d` | vy ±0.1 m/s (de costado) | | `espacio` | comando (0,0,0) — frenar |
| `x` | salir (deja el robot en amortiguación) | | `Ctrl+C` | ídem x |

Sin teclado (corridas fijas / headless): `python scripts/03_walk_policy.py --cmd 0.4 0 0 --dur 10`.
Flags: `--device cuda` (innecesario: CPU infiere en <1 ms), `--real IFACE` (robot real,
libera el sport mode solo — ¡primero en arnés!).

⚠️ Lecciones de deployment aprendidas debugueando el "temblor" (todas aplicadas
por `setup.sh` / el script):

1. **`SIMULATE_DT = 0.002`** (era 0.005): el culpable principal. Con paso de
   física grueso los contactos pie-piso se integran mal y cualquier policy
   tiembla (roll std 0.93 → 0.26 rad/s solo con este cambio).
2. **Escena plana**: la escena go2 de unitree_mujoco trae cordones (x=1.2) y una
   escalera (x=2.3+) — el robot se los llevaba puestos caminando hacia +x.
   `setup.sh` los saca; para recuperarlos: `git -C external/unitree_mujoco checkout .`
3. **Re-publicar lowcmd a 500 Hz** aunque la policy decida a 50: el bridge Python
   recalcula el PD solo al recibir mensaje (el robot real lo hace a kHz solo).
4. **Timing con agenda absoluta**: `sleep(resto)` en WSL corre ~8% lento
   (policy a 46 Hz en vez de 50).

El gap sim2sim residual (entrenadas en IsaacGym/Lab, corren en MuJoCo) sigue
existiendo — la solución de fondo sigue siendo entrenar en el mismo motor (mjlab).

Después, notebooks (kernel = Python de `~/.venvs/go2` en WSL):
- [01_explore_lowstate.ipynb](notebooks/01_explore_lowstate.ipynb) — señales básicas de una grabación (stand).
- [02_explore_gait.ipynb](notebooks/02_explore_gait.ipynb) — autopsia del trote: patrón diagonal, frecuencia de zancada (FFT), temblor cuantificado.

Tip del viewer de MuJoCo: Ctrl+click derecho y arrastrar aplica fuerzas al robot
(empujalo mientras está parado y mirá la IMU).

### Estructura

```
docs/        contexto: simuladores, ecosistema DDS, arquitectura, GLOSARIO
systems/    fase 2: la jetson y el servidor simulados + degradador de wifi
external/    clones (gitignored): unitree_mujoco, unitree_sdk2_python
scripts/     fase 1: loops de control · fase 2: lab_up.sh / lab_down.sh
notebooks/   fase 1: datos grabados · fase 2: 03_ros2_observatory (kernel en VM)
data/        grabaciones .npz (gitignored)

Convención: nombres de archivos, carpetas e identificadores en inglés;
comentarios, documentación y notas en español.
```

### Contra el robot real (cuando toque)

1. Ethernet al RJ45 del Go2, IP estática `192.168.123.99/24` en tu lado.
   En WSL2 hace falta puentear la interfaz (mirrored networking en
   `.wslconfig`) o pasar por USB-ethernet — a resolver cuando llegue el momento.
2. `python scripts/00_read_lowstate.py --real <iface>` para verificar conexión.
3. Para bajo nivel, `01_stand.py --real` ya libera el sport mode a bordo
   (`MotionSwitcherClient`) antes de comandar. **Primero con el robot colgado
   del arnés.**
4. Alto nivel (`SportClient.Move()` etc.) solo existe contra el robot real —
   probar ahí, no en sim.

### Próximos pasos candidatos

- Oscilación senoidal de una articulación (sentir kp/kd y el control PD).
- Leer `rt/sportmodestate` en sim (ground truth del cuerpo — odometría gratis).
- Comparar `SIMULATE_DT` / frecuencias efectivas; probar la versión C++ del sim.
- mjlab / unitree_rl_mjlab para entrenar locomoción con la GPU.
