# go2-lab

Entorno de trabajo sim → real para el Unitree Go2 EDU (y a futuro G1 EDU).
Contexto completo en [docs/00_mapa_simuladores.md](docs/00_mapa_simuladores.md) y
[docs/01_unitree_ecosistema_datos.md](docs/01_unitree_ecosistema_datos.md).

## Las piezas y qué hace cada una

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

## Entorno (WSL2)

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
      -e external/unitree_sdk2_python mujoco pygame matplotlib ipykernel jupyterlab
  ```
  (o directamente `bash scripts/setup.sh`, que hace todo esto)

## Uso (dos terminales WSL)

```bash
source ~/.venvs/go2/bin/activate

# Terminal A: simulador (abre ventana de MuJoCo con el Go2)
bash scripts/sim.sh

# Terminal B: scripts contra el sim
python scripts/00_read_lowstate.py        # ver telemetría en vivo
python scripts/01_stand.py                # pararse y agacharse (control PD)
python scripts/02_record_lowstate.py      # grabar datos a data/*.npz
```

Después: [notebooks/01_explore_lowstate.ipynb](notebooks/01_explore_lowstate.ipynb)
para graficar lo grabado (kernel = Python de `~/.venvs/go2` en WSL).

Tip del viewer de MuJoCo: Ctrl+click derecho y arrastrar aplica fuerzas al robot
(empujalo mientras está parado y mirá la IMU).

## Estructura

```
docs/        contexto: mapa de simuladores, ecosistema DDS/datos de Unitree
external/    clones (gitignored): unitree_mujoco, unitree_sdk2_python
scripts/     loops de control y utilidades, numerados en orden didáctico
notebooks/   exploración de datos grabados
data/        grabaciones .npz (gitignored)
```

## Contra el robot real (cuando toque)

1. Ethernet al RJ45 del Go2, IP estática `192.168.123.99/24` en tu lado.
   En WSL2 hace falta puentear la interfaz (mirrored networking en
   `.wslconfig`) o pasar por USB-ethernet — a resolver cuando llegue el momento.
2. `python scripts/00_read_lowstate.py --real <iface>` para verificar conexión.
3. Para bajo nivel, `01_stand.py --real` ya libera el sport mode a bordo
   (`MotionSwitcherClient`) antes de comandar. **Primero con el robot colgado
   del arnés.**
4. Alto nivel (`SportClient.Move()` etc.) solo existe contra el robot real —
   probar ahí, no en sim.

## Próximos pasos candidatos

- Oscilación senoidal de una articulación (sentir kp/kd y el control PD).
- Leer `rt/sportmodestate` en sim (ground truth del cuerpo — odometría gratis).
- Comparar `SIMULATE_DT` / frecuencias efectivas; probar la versión C++ del sim.
- mjlab / unitree_rl_mjlab para entrenar locomoción con la GPU.
