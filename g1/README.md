# G1: el sistema de la demo

El robot camina, ve, navega solo y ejecuta misiones dadas en castellano.

```bash
# el robot (necesita GPU: corre Isaac)
bash run_g1.sh wbc 29dof 0 "--camera --scene"

# las computadoras simuladas y su red
cd ../systems && bash systems_up.sh

# las capas locales del robot (dentro del contenedor jetson)
python3 mobility_authority.py  # único dueño de /cmd_vel
python3 stand_hold.py          # corrige la deriva durante una espera
python3 skills/go_to.py        # navegación
python3 skills/object_detector.py   # detector neuronal local
python3 skills/detection_adapter.py # cajas, color y recorte del reloj
python3 agent/agent.py         # ejecutor local de la misión

# darle una misión
ros2 topic pub --once /g1/mission std_msgs/msg/String \
  "{data: mira la hora y trae el objeto de la mesa que corresponda}"
```

El lanzador requiere WBC-AGILE en `~/go2-lab/WBC-AGILE` y verifica el commit
`7259792cf10803aab814d101134d493d24c8f22f`. Una versión distinta no arranca
hasta volver a pasar las pruebas físicas.

## Qué funciona hoy

| Pieza | Estado |
|---|---|
| Locomoción (camina, gira) | funciona — conjunto NVIDIA AGILE verificado |
| Cuerpo con brazos (29 articulaciones) | funciona — modelo oficial del G1 |
| Control de brazos (poses) | funciona — `/g1/arm_pose` |
| Carga en las manos | pendiente de repetir con AGILE; ver `PAYLOAD_TEST_PLAN.md` |
| Cámara de cabeza | funciona — óptica oficial Unitree, 640×480 a 3 Hz |
| Detector local RT-DETR | integrado; reloj 3/3, mesa visible pero debajo del umbral |
| Búsqueda visual abierta | Grounding DINO reconoce la mesa; falta ubicarlo sin afectar el control |
| Lectura del reloj en servidor | funciona — 3/3 limpia y 3/3 con red mala |
| Navegación a un punto | funciona — `/g1/goal` → `/g1/nav_status` |
| Corte del servidor | funciona — la misión falla explícitamente y el robot queda local |
| Ejecutor de misión | en migración a la misión de mesas roja/azul y regreso a `home` |
| Agarrar | pendiente (lo hará un VLA) |

La misión vigente está definida en [`DEMO_TARGET.md`](DEMO_TARGET.md): guardar
el punto de inicio, leer el reloj, encontrar una mesa roja o azul que no está
registrada por nombre, tomar su objeto y regresar al inicio. Las personas de la
versión anterior quedaron fuera del alcance actual.

## La arquitectura

```
      SERVIDOR EXTERNO                    JETSON A BORDO
 [ intelligence_service.py ] <---HTTP--- [ agent/agent.py ]
       modelo visual                      ejecuta la misión
              ^                                  |
              | recorte JPEG              /g1/goal /g1/arm_pose
              |                                  |
              +------------ [ object_detector.py + adapter ] [ go_to.py ]
                                                   |
                                    /g1/cmd_vel/navigation
                                                   |
                            [ stand_hold.py ] -> [ mobility_authority.py ]
                                                          |
                                                      /cmd_vel
                                                          |
    +-------------------+--------------------------------+
    |  EL ROBOT (g1_robot.py)                            |
    |   física + locomoción, lazo de control cerrado      |
    |   + control de brazos + cámara                      |
    +-------------------+--------------------------------+
              /g1/odom  /g1/joint_states  /g1/head_cam/image
```

| Topic | Mensaje | Quién lo produce |
|---|---|---|
| `/g1/mission` | String | vos (o el reconocimiento de voz, más adelante) |
| `/g1/goal` | PoseStamped | el agente |
| `/g1/nav_status` | String | la navegación |
| `/g1/object_detections` | Detection2DArray | el detector neuronal |
| `/g1/detections` | String (JSON) | el adaptador de la demo |
| `/g1/clock_crop/compressed` | CompressedImage | el adaptador local |
| `/g1/arm_pose` | String | el agente |
| `/g1/cmd_vel/{stand,navigation,manual,test}` | Twist | cada fuente identificada |
| `/g1/mobility/request` | String (JSON transitorio) | quien solicita o libera movilidad |
| `/g1/mobility/status` | String (JSON) | el árbitro de movilidad |
| `/cmd_vel` | Twist | sólo `mobility_authority.py` |
| `/g1/odom`, `/g1/joint_states` | Odometry, JointState | el robot |
| `/g1/head_cam/image` | Image | el robot |

Cada pieza se puede reemplazar sin tocar las demás mientras respete su contrato:
la navegación por Nav2, RT-DETR por otro detector que publique las mismas
cajas, el planificador de reglas
por un modelo de lenguaje, el robot simulado por el real.

## Los archivos

| Archivo | Qué hace |
|---|---|
| `g1_robot.py` | el robot: física, locomoción, brazos, cámara. Un proceso, lazo cerrado |
| `locomotion.py` | controladores intercambiables (NVIDIA AGILE / anterior / diagnóstico) |
| `arm_control.py` | control de brazos por poses con nombre |
| `perception.py` | la cámara de la cabeza y su publicación |
| `g1_asset.py` | los cuerpos disponibles (12 y 29 articulaciones) y sus actuadores |
| `demo_scene.py` | la habitación, los objetos y el reloj digital |
| `skills/go_to.py` | navegación a un punto |
| `mobility_authority.py` | concede la movilidad a una sola fuente y alimenta `/cmd_vel` |
| `stand_hold.py` | mantiene una pose durante una espera; no navega |
| `skills/object_detector.py` | RT-DETR local con salida estándar de ROS 2 |
| `skills/detection_adapter.py` | conserva cuadros acotados, clasifica color y recorta el reloj |
| `agent/agent.py` | ejecutor local de la misión y cliente del servidor |
| `../systems/server/intelligence_service.py` | modelos lentos en el servidor externo |
| `run_g1.sh` | lanzador |

## Lo que costó acertar (para no repetirlo)

**Cuerpo, motores, entradas y policy forman un conjunto.** Mezclar la policy
anterior del G1 simplificado con nuestro cuerpo convertido producía unos
`9 cm/s` de deriva. La base actual importa desde NVIDIA AGILE el G1 oficial,
sus motores simulados, el descriptor de 80 entradas y la policy recurrente.
Copiar solamente un `.pt` volvería a repetir el error.

**Física a 200 Hz y control a 50 Hz.** Son las frecuencias ejecutables de la
policy de altura y velocidad elegida en AGILE. Se conserva la configuración
oficial aunque comentarios antiguos de ese repositorio mencionen 500 Hz.

**Escribir el estado inicial explícitamente** después de crear la escena, o el
articulado nace colapsado.

**Nada de "asentamiento" con las piernas rígidas**: un bípedo rígido se cae. La
policy tiene que controlar desde el primer instante.

**Física y locomoción en el mismo proceso.** Separarlas en procesos asíncronos
abre el lazo de control en el tiempo y el bípedo se cae. La modularidad va en
la orden de velocidad. En esta demo, un árbitro exclusivo deja una única salida
en `/cmd_vel`; en el G1 real esa salida podrá alimentar la interfaz de Unitree.

**No se puede esperar dentro de un callback de ROS** ("Executor is already
spinning"): las tareas largas van en un hilo aparte que lee lo que los
callbacks actualizan.

**Los destinos de navegación son poses de aproximación**, no coordenadas del
objeto. Si el destino es el centro de la mesa, el robot camina contra el mueble
y la navegación nunca da por cumplido el objetivo.

**Los plazos medidos en tiempo real no sirven** con el simulador corriendo al
20 %: recorrer 8 m lleva más de dos minutos de reloj de pared. La solución de
fondo es el reloj simulado de ROS 2 (`/clock` + `use_sim_time`).

**Los modelos lentos no controlan el cuerpo.** La Jetson recorta la imagen del
reloj y la manda al servidor por HTTP. El servidor consulta el modelo visual y
devuelve datos validados. Si la red o el proveedor fallan, el paso se aborta;
equilibrio, espera y autoridad de movilidad siguen funcionando localmente.

## Lo siguiente

La cámara, sus memorias acotadas y lo que muestra el tablero están explicados
en [`PERCEPTION_ARCHITECTURE.md`](PERCEPTION_ARCHITECTURE.md).

1. **Integrar la búsqueda puntual con Grounding DINO** fuera de los lazos de
   control y probar el efecto de red y cómputo.
2. **Guardar `home` y regresar** sin carga.
3. **Buscar la mesa roja o azul** sin pasarle una coordenada por nombre.
4. **Rediseñar `transporte`**: quieto es estable, pero caminando sesga el rumbo;
   probar una postura más cercana a neutral y luego la escalera de cargas.
5. **Reloj simulado** (`/clock` + `use_sim_time`) para medir plazos en tiempo
   de simulación.
6. **El agarre** con un VLA entrenado por nosotros.
7. **Voz y planificador semántico** después de cerrar las capacidades físicas.
