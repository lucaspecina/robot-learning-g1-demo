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

### Resultado de quietud

La referencia intacta se ejecutó el 31 de julio de 2026 sobre la T4, con tres
repeticiones de diez segundos simulados:

| Repetición | Desplazamiento máximo | Altura mínima | Inclinación máxima |
|---|---:|---:|---:|
| 1 | `3,23 cm` | `72,13 cm` | `9,06°` |
| 2 | `6,75 cm` | `71,92 cm` | `7,00°` |
| 3 | `6,74 cm` | `71,92 cm` | `6,97°` |

Las tres fueron aceptadas y ninguna terminó antes de tiempo. El entorno
confirmó `43` articulaciones en total, `14` dedos y una acción externa de
`32` valores: `28` para muñecas y dedos, y `4` para velocidad X/Y, giro y
altura. Esto demuestra que la tarea oficial funciona en esta VM; no demuestra
todavía que nuestra integración tenga el mismo comportamiento.

Para caminar sin generar una fuerza artificial, las muñecas se conservan
respecto de la pelvis y sus objetivos mundiales se actualizan en cada paso.
Dejarlas fijas en el mundo mientras el robot avanza obligaría a los brazos a
quedarse atrás y contaminaría la prueba de piernas.

### Resultado de caminata y frenado

El mando oficial de los controles de mano convierte el rango `[-1, 1]` a
`[-0,5, 0,5]` para avance y costado. Por eso la prueba usó `0,30`, un valor
válido dentro del contrato oficial, durante cuatro segundos, y luego pidió
velocidad cero durante otros cuatro.

| Repetición | Avance | Error lateral máximo | Recorrido al frenar | Velocidad final |
|---|---:|---:|---:|---:|
| 1 | `13,88 cm` | `1,34 cm` | `6,00 cm` | `7,78 cm/s` |
| 2 | `13,77 cm` | `2,80 cm` | `14,51 cm` | `7,19 cm/s` |
| 3 | `11,94 cm` | `2,77 cm` | `18,52 cm` | `0,57 cm/s` |

Las tres repeticiones mantuvieron al robot de pie, con altura mínima entre
`66,08` y `67,19 cm` e inclinación máxima menor que `13,79°`. La dirección
fue correcta y el error lateral fue pequeño, pero ninguna alcanzó el criterio
externo de `50 cm` de avance; una superó por `3,52 cm` el límite de frenado.
Esto no invalida la referencia oficial: demuestra que la policy equilibra y
responde al mando, no que sea un controlador preciso de posición ni que siga
perfectamente la velocidad solicitada. La navegación debe cerrar ese error.

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
- [Conversión oficial del mando de locomoción del G1](https://github.com/isaac-sim/IsaacLab/blob/v2.3.2/source/isaaclab/isaaclab/devices/openxr/retargeters/humanoid/unitree/g1_motion_controller_locomotion.py)
- [Dependencia oficial `numpy<2` de Isaac Lab 2.3.2](https://github.com/isaac-sim/IsaacLab/blob/v2.3.2/source/isaaclab/setup.py)
- [Flujo de teleoperación e imitación fijado en Isaac Lab 2.3.2](https://github.com/isaac-sim/IsaacLab/blob/v2.3.2/docs/source/overview/imitation-learning/teleop_imitation.rst)
- [Configuración exacta del control de muñecas, dedos y cintura](https://github.com/isaac-sim/IsaacLab/blob/v2.3.2/source/isaaclab_tasks/isaaclab_tasks/manager_based/locomanipulation/pick_place/configs/pink_controller_cfg.py)
- [Suite oficial de Unitree con tareas móviles y manos](https://github.com/unitreerobotics/unitree_sim_isaaclab)
- [Configuraciones oficiales de cámaras de muñeca de Unitree](https://github.com/unitreerobotics/unitree_sim_isaaclab/blob/main/tasks/common_config/camera_configs.py)
