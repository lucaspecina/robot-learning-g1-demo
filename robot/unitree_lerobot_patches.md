# Cambios a unitree_lerobot

[`unitree_lerobot`](https://github.com/unitreerobotics/unitree_lerobot) es el
repositorio oficial de Unitree para convertir grabaciones de teleoperación al
formato de LeRobot, entrenar y evaluar en el G1. Lo seguimos tal como indica
su README; estos son los únicos cambios que hicimos, y por qué. **No es el
flujo oficial sin modificar.**

Todos se hicieron en la copia del robot y dejan un respaldo `.orig` donde
correspondía, para poder revertirlos.

## 1. Configuración de una sola cámara

**Archivo:** `unitree_lerobot/utils/constants.py`.

La configuración de serie para el G1 con manos Brainco espera cuatro cámaras.
Nuestra teleoperación graba sólo la de la cabeza (`color_0`), así que la
conversión fallaba. Se agregó una configuración que reusa los motores de la
original y declara una cámara:

```python
# Our teleoperation setup records a single head camera (color_0) instead of the
# four cameras the stock Brainco config expects. The camera is named
# cam_right_high because that is the input key the trained policy asks for.
G1_BRAINCO_1CAM_CONFIG = RobotConfig(
    motors=G1_BRAINCO_CONFIG.motors,
    cameras=["cam_right_high"],
    camera_to_image_key={"color_0": "cam_right_high"},
    json_state_data_name=["left_arm.qpos", "right_arm.qpos", "left_ee.qpos", "right_ee.qpos"],
    json_action_data_name=["left_arm.qpos", "right_arm.qpos", "left_ee.qpos", "right_ee.qpos"],
)
```

y se la registró en `ROBOT_CONFIGS` como `"Unitree_G1_Brainco_1Cam"`.

## 2. Tamaño de imagen

**Archivo:** `convert_unitree_json_to_lerobot.py`.

El conversor declara imágenes de 480×640; nuestras grabaciones son de 360×640.

```python
# Our recordings are 360x640; the stock value assumed 480x640.
"shape": (360, 640, 3),
```

En este repo, ese conversor se reusa desde
[`toolkit/dataset/convert_unitree.py`](../toolkit/dataset/convert_unitree.py),
que lee el tamaño de la imagen de las grabaciones en vez de parchear el
archivo, y registra la configuración de una cámara sin tocar `constants.py`.

## 3. Modo simulacro en la evaluación

**Archivo:** `eval_robot/eval_g1.py` (respaldo `eval_g1.py.orig`).

Con `send_real_robot` en falso, el script no lleva el robot a la pose inicial
ni envía órdenes a brazos y manos; registra lo que la política habría mandado
y lo guarda en `dry_run_log.npz`.

**Estado: no funciona todavía.** El cliente de imágenes del repositorio cambió
de interfaz y el script no se actualizó; hay que adaptarlo antes de la prueba
en el robot.

## 4. Configuración del modelo

**Archivo:** `config.json` del modelo entrenado (respaldo `config.json.orig`).

Se quitaron los campos `use_peft` y `pretrained_revision`, que impedían cargar
la configuración con la versión de LeRobot instalada (0.4.1).

## Regla que sale de estos cambios

La política de LeRobot no normaliza internamente. Siempre se usa con su
preprocesador y postprocesador (`make_pre_post_processors`); sin ellos, las
predicciones y cualquier métrica sobre ellas son inválidas.
