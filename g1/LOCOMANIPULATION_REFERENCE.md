# Referencia oficial de locomoción y manipulación

Fecha de verificación: 2026-07-30.

## Qué encontró la investigación

Isaac Lab `v2.3.2` contiene la tarea oficial
`Isaac-PickPlace-Locomanipulation-G1-Abs-v0`. Es la referencia más cercana a
nuestra demo porque combina, dentro del mismo G1:

- una policy AGILE que recibe velocidad frontal, lateral, giro y altura, y
  controla las doce articulaciones de las piernas;
- un controlador de **cinemática inversa** para la parte superior. Cinemática
  inversa significa pedir “poné la muñeca en esta posición y orientación” y
  calcular los ángulos de hombros, codos, muñecas y cintura que lo logran;
- catorce articulaciones de dedos;
- control desde dispositivos XR para brazos, dedos, caminata, giro y altura.

La física oficial corre a `200 Hz` y el entorno entrega nuevas órdenes a
`50 Hz`. El controlador de piernas observa velocidades del cuerpo, gravedad,
las 29 articulaciones sin contar dedos y su orden anterior. Por eso puede
compensar el movimiento de brazos y cintura.

## Qué NO resuelve esa tarea

No es un robot autónomo que mira una mesa y decide cómo agarrar:

- el entorno base recibe directamente posiciones del objeto conocidas por el
  simulador;
- la parte superior recibe objetivos de muñeca y dedos provenientes de una
  persona, una trayectoria grabada o una policy entrenada;
- no incluye nuestro flujo cámara → detección → lectura → plan;
- no reemplaza navegación, mapa, seguridad ni autoridad de movimiento.

Por lo tanto es una referencia oficial excelente para **cuerpo, manos y
contratos de control**, pero no una solución completa de la demo.

## Diferencias comprobadas con nuestra integración

| Parte | Isaac Lab oficial | Demo actual |
|---|---|---|
| Piernas | `agile_locomotion.pt` del ejemplo | policy recurrente desplegable de WBC-AGILE |
| Orden inferior | velocidad X/Y, giro y altura | el mismo tipo de orden |
| Parte superior | objetivos continuos de ambas muñecas y dedos | poses articulares con nombre |
| Manos | 14 articulaciones de dedos | desactivadas en el cuerpo de AGILE |
| Percepción | estado exacto del objeto dentro del entorno | cámara RGB con profundidad y detectores |
| Navegación | se agrega en el flujo de generación de datos | ROS 2 con dueño único del movimiento |

Las dos policies de piernas pertenecen a AGILE, pero no son el mismo archivo
ni el mismo contrato. No se puede copiar sólo el cuerpo o sólo la salida de
una dentro de la otra y llamarlo “oficial”.

## Prueba intacta antes de integrar

`tools/evaluate_official_locomanipulation.py` carga la configuración desde
Isaac Lab sin cambiarla, conserva las muñecas y dedos en su postura inicial,
envía velocidad y giro cero con la altura oficial de cadera de `0,72 m`, y
sólo observa. Todo cero no sería una orden neutral: pediría altura de cadera
cero y dos orientaciones de muñeca inválidas.
`tools/run_official_locomanipulation_eval.sh` fija explícitamente las rutas de
`v2.3.2`, porque la VM también tiene una rama `main` instalada y mezclar ambas
produjo un cierre silencioso.

La primera ejecución reveló otro problema de entorno antes de llegar a la
física: la instalación global de la VM tenía NumPy `2.4.6`, mientras que
Isaac Lab `v2.3.2` declara `numpy<2`. Pinocchio, la biblioteca matemática que
usa el controlador de brazos, fue compilada para NumPy 1 y fallaba al cargar.
La referencia usa NumPy `1.26.4` desde una carpeta aislada; no se degradó el
entorno global de la demo. También carga Pinocchio antes de abrir Isaac y pasa
`--enable_pinocchio`, siguiendo los comandos oficiales.

La primera prueba declarada es:

- tres repeticiones;
- diez segundos simulados por repetición;
- desplazamiento máximo `0,10 m`;
- altura mínima `0,65 m`;
- inclinación máxima `20°`;
- ningún final prematuro.

El informe también debe confirmar la ruta exacta del código importado, la
cantidad de articulaciones y la existencia real de los catorce dedos. Después
se hará caminata y frenado; una prueba quieta no valida locomoción.

## Orden de integración recomendado

1. Reproducir quietud, caminata y frenado en la tarea oficial intacta.
2. Confirmar visualmente manos, postura y sentido de cada orden.
3. Incorporar una variante de manos aislada en nuestra base y repetir las
   tres pruebas físicas.
4. Incorporar el controlador oficial de muñecas y dedos detrás de una
   interfaz intercambiable; conservar las poses con nombre sólo para estados
   seguros como reposo y transporte.
5. Recién con la mano física elegida, agregar las cámaras de muñeca que usa
   Unitree y medir campo visual y costo de cómputo.
6. Mantener `grasp_object` como no disponible hasta tener posición y
   orientación completas del objeto, trayectoria sin colisiones, cierre
   confirmado y prueba de carga.

## Fuentes oficiales

- [Configuración G1 de locomoción y manipulación de Isaac Lab](https://github.com/isaac-sim/IsaacLab/blob/v2.3.2/source/isaaclab_tasks/isaaclab_tasks/manager_based/locomanipulation/pick_place/locomanipulation_g1_env_cfg.py)
- [Dependencia oficial `numpy<2` de Isaac Lab 2.3.2](https://github.com/isaac-sim/IsaacLab/blob/v2.3.2/source/isaaclab/setup.py)
- [Flujo de teleoperación e imitación fijado en Isaac Lab 2.3.2](https://github.com/isaac-sim/IsaacLab/blob/v2.3.2/docs/source/overview/imitation-learning/teleop_imitation.rst)
- [Configuración exacta del control de muñecas, dedos y cintura](https://github.com/isaac-sim/IsaacLab/blob/v2.3.2/source/isaaclab_tasks/isaaclab_tasks/manager_based/locomanipulation/pick_place/configs/pink_controller_cfg.py)
- [Suite oficial de Unitree con tareas móviles y manos](https://github.com/unitreerobotics/unitree_sim_isaaclab)
- [Configuraciones oficiales de cámaras de muñeca de Unitree](https://github.com/unitreerobotics/unitree_sim_isaaclab/blob/main/tasks/common_config/camera_configs.py)
