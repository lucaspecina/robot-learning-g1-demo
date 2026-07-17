# Mapa de simuladores para robótica (foco: robots con patas)

> Estado: julio 2026. Contexto: Unitree Go2 EDU (ya lo tengo) + G1 EDU (en compra).

## 1. Las 4 capas del ecosistema

La confusión típica es mezclar cosas que viven en niveles distintos:

1. **Motor de física (physics engine)**: integra las ecuaciones de movimiento — dinámica de cuerpos articulados, colisiones, contactos, fricción. Sin GUI, sin sensores. Ej: MuJoCo (el core), PhysX, Bullet, Newton. La calidad del contacto pie-suelo de esta capa es lo que determina si una política de locomoción transfiere al robot real.

2. **Simulador completo**: envuelve un motor de física y agrega render 3D, sensores simulados (cámaras, lidar, IMU), editor de escenas, importación de modelos (URDF/MJCF/USD). Ej: Isaac Sim, Gazebo, Webots. MuJoCo es un híbrido: motor + visualizador + sensores básicos, por eso se usa como "simulador" directo.

3. **Framework de RL / robot learning**: encima del simulador; provee la abstracción de "entorno": miles de envs paralelos en GPU, observaciones/recompensas modulares, domain randomization, terrenos. Ej: Isaac Lab (sobre Isaac Sim), mjlab y MuJoCo Playground (sobre MuJoCo Warp/MJX), legged_gym (legacy).

4. **Middleware / bridge / deployment**: conecta la sim con el stack del robot real, para que el MISMO código corra en sim y en hardware. Ej: ROS 2 + ros_gz, y en el mundo Unitree: **`unitree_mujoco`** (expone en MuJoCo la misma API DDS que el robot físico — la pieza clave para mí).

**Flujo típico 2026 para patas**: entrenar en capa 3 (Isaac Lab o mjlab) → validar el checkpoint en OTRO motor (sim2sim en MuJoCo vía unitree_mujoco) → deploy vía SDK/DDS al robot. El "dual-sim validation" es práctica estándar porque los modos de falla de cada motor están poco correlacionados.

## 2. Los jugadores (estado 2026)

### MuJoCo (+ MJX + MuJoCo Warp) — DeepMind
- El estándar académico para locomoción. Contactos excelentes (lo mejor para sim-to-real de patas).
- Tres sabores: clásico (CPU, corre en cualquier laptop), MJX (JAX), **MuJoCo Warp** (GPU NVIDIA, la vía principal a futuro, speedups de 100-400x vs MJX).
- **Es el simulador oficial de Unitree** (`unitree_mujoco` soporta Go2 y G1 con la misma API DDS del robot real).
- Ecosistema: MuJoCo Menagerie (MJCF oficiales de Go2/G1/H1), MuJoCo Playground (envs de locomoción con sim-to-real demostrado).
- Débil en: percepción/lidar realista. Apache 2.0.

### NVIDIA Isaac Sim / Isaac Lab
- **Isaac Gym está deprecado**; todo se consolidó en **Isaac Lab** (framework RL) sobre **Isaac Sim** (simulador, render RTX fotorrealista, física PhysX → migrando a Newton).
- El más completo en sensores: RTX lidar por ray tracing, cámaras fotorrealistas, datos sintéticos. "Casi default" para RL de humanoides a escala.
- Contras: instalación pesada, curva empinada, pide RTX 4080+ / 16GB+ VRAM. En plena transición (Isaac Lab 2.3.1 estable, main congelado migrando a Newton; Isaac Sim 6.0 en early release) — momento algo inestable para entrar.

### Newton (NVIDIA + DeepMind + Disney, Linux Foundation)
- Motor de física GPU nuevo, **usa MuJoCo Warp como solver principal**. 1.0 GA en GTC 2026.
- Es la convergencia del ecosistema: Isaac Lab y el mundo MuJoCo van a compartir física. No se elige directamente: llega vía Isaac Lab o MuJoCo Warp.

### mjlab (Berkeley: Zakka, Liao, Sreenath, Abbeel — 2026)
- "APIs de Isaac Lab + física MuJoCo Warp": ligero, `pip install`, PyTorch zero-copy. Requiere GPU NVIDIA para entrenar.
- **Unitree lo adoptó oficialmente**: `unitree_rl_mjlab` soporta Go2, G1, H1_2, etc. con pipeline de deploy real. Es "todo lo bueno de Isaac Lab sin instalar Isaac Sim".

### Genesis
- Simulador multi-física en Python (rígidos + fluidos + soft-body), muy activo (v1.2.2, jul 2026), API amigable, multi-backend (CUDA/ROCm/Metal).
- Tutorial de locomoción Go2 muy didáctico. Ojo: sus claims de velocidad iniciales fueron desmentidos por benchmarks independientes. Menos historial sim-to-real que MuJoCo/Isaac.

### Gazebo (moderno, ex-Ignition)
- El simulador del ecosistema ROS 2 (Gazebo Classic llegó a EOL en 2025; hoy: Harmonic/Jetty LTS).
- Para testear el stack ROS 2 completo (nav, SLAM, percepción) — NO para entrenar RL (CPU, un env a la vez). Soporte Go2 solo comunitario.

### Los que ya no: PyBullet (mantenimiento mínimo, no arrancar acá en 2026), Brax (su física fue absorbida por MJX; hoy solo queda su librería de training JAX), legged_gym/Isaac Gym (legacy, migrado a Isaac Lab), Webots/CoppeliaSim (educación/nicho, no para patas serias).

## 3. Tabla resumen

| Simulador | Capa | Física patas | GPU masivo | Sensores | Curva | Estado 2026 |
|---|---|---|---|---|---|---|
| MuJoCo clásico | Motor+viz | Excelente | No (CPU) | Básicos | Baja | Muy activo, estándar |
| MuJoCo Warp | Motor GPU | Excelente | Sí | Básicos | Media | Beta, la vía a futuro |
| mjlab | Framework RL | Excelente | Sí | Los de MuJoCo | Media | Nuevo, adoptado por Unitree |
| Isaac Sim/Lab | Sim + Framework | Buena→Exc. (Newton) | Sí | Los mejores (RTX lidar, cám) | Alta | En transición a Newton |
| Genesis | Sim multi-física | Buena (menos validada) | Sí (multi-vendor) | Cám/lidar/IMU | Baja-media | Muy activo, emergente |
| Gazebo | Simulador | Media | No | Muy buenos + ROS 2 | Media | Activo; solo integración |
| PyBullet | Motor+API | Media | No | Básicos | Muy baja | Casi abandonado |

## 4. Quién se usa para qué

- **RL de locomoción**: Isaac Lab vs stack MuJoCo Warp (mjlab / Playground), compitiendo de igual a igual. Genesis tercero emergente.
- **Testing de software / control clásico / integración ROS 2**: Gazebo.
- **Percepción / sensores fotorrealistas**: Isaac Sim, sin discusión.
- **Sim-to-real con Unitree**: pipeline oficial = entrenar (unitree_rl_lab en Isaac Lab, o unitree_rl_mjlab en mjlab) → validar en `unitree_mujoco` (misma API DDS que el robot) → deploy por ethernet.

## 5. Lectura estratégica

Todo el ecosistema converge hacia **física MuJoCo-Warp en GPU** (vía Newton en Isaac Lab, vía mjlab en el mundo DeepMind/Unitree). Para Go2+G1 hoy, la ruta de menor fricción y mayor alineación con Unitree es **MuJoCo + unitree_mujoco** (para entender datos/conexiones) y **mjlab/unitree_rl_mjlab** (para entrenar), con Isaac Lab como opción si necesito percepción RTX.

## Fuentes principales
- https://github.com/unitreerobotics/unitree_mujoco · https://github.com/unitreerobotics/unitree_rl_mjlab · https://github.com/unitreerobotics/unitree_rl_lab
- https://mujoco.readthedocs.io/en/latest/mjwarp/ · https://github.com/google-deepmind/mujoco_playground · https://github.com/mujocolab/mjlab
- https://developer.nvidia.com/newton-physics · https://github.com/isaac-sim/IsaacLab/discussions/4339
- https://github.com/Genesis-Embodied-AI/Genesis · https://gazebosim.org/docs/latest/releases/
