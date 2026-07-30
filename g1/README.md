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
python3 skills/open_vocabulary_detector.py # búsqueda puntual en el servidor
python3 skills/table_localizer.py   # lleva una mesa detectada al mapa
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
| Habitación física | funciona — cuatro paredes con colisión, alineadas con el tablero |
| Cámara de cabeza | funciona — color, profundidad y calibración sincronizados |
| LiDAR simulado | experimental; aislado funciona, integrado aún entrega nubes vacías |
| Detector local RT-DETR | integrado; reloj 3/3, mesa visible pero debajo del umbral |
| Búsqueda visual abierta | integrada por pedido; roja y azul 3/3, red mala y corte probados |
| Mesa visual a punto del mapa | funciona — roja y azul caen sobre su superficie real |
| Lectura del reloj en servidor | funciona — 3/3 limpia y 3/3 con red mala |
| Navegación a un punto | funciona — `/g1/goal` → `/g1/nav_status` |
| Corte del servidor | funciona — la misión falla explícitamente y el robot queda local |
| Ejecutor de misión | integrado hasta la búsqueda; se bloquea explícitamente porque falta el barrido visual activo |
| Agarrar | pendiente (lo hará un VLA) |

La misión vigente está definida en [`DEMO_TARGET.md`](DEMO_TARGET.md): guardar
el punto de inicio, leer el reloj, encontrar una mesa roja o azul que no está
registrada por nombre, tomar su objeto y regresar al inicio. Las personas de la
versión anterior quedaron fuera del alcance actual.

## La arquitectura

```
      SERVIDOR EXTERNO                    JETSON A BORDO
 [ intelligence_service.py ] <---HTTP--- [ agent/agent.py ]
       modelos visuales                    ejecuta la misión
              ^                                  |
              | recorte/cuadro JPEG        /g1/goal /g1/arm_pose
              |                                  |
              +-- [ open_vocabulary_detector.py ] [ object_detector.py ]
                         |                 \             /
                 [ table_localizer ]        [ adapter ] [ go_to.py ]
                         |                             |
              /g1/table_detections_3d      /g1/cmd_vel/navigation
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
| `/g1/mission_state` | String (JSON transitorio) | estado, pasos y decisiones verificables de la misión |
| `/g1/model_events` | String (JSON transitorio) | entrada, texto literal y salida validada de cada modelo remoto |
| `/g1/goal` | PoseStamped | el agente |
| `/g1/nav_status` | String | la navegación |
| `/g1/object_detections` | Detection2DArray | el detector neuronal |
| `/g1/perception/search_request` | String (JSON) | agente o verificador |
| `/g1/open_vocabulary_detections` | Detection2DArray | búsqueda puntual |
| `/g1/table_detections_3d` | Detection3DArray | localizador de mesas |
| `/g1/perception/search_status` | String (JSON) | búsqueda puntual |
| `/g1/detections` | String (JSON) | el adaptador de la demo |
| `/g1/clock_crop/compressed` | CompressedImage | el adaptador local |
| `/g1/arm_pose` | String | el agente |
| `/g1/cmd_vel/{stand,navigation,manual,test}` | Twist | cada fuente identificada |
| `/g1/mobility/request` | String (JSON transitorio) | quien solicita o libera movilidad |
| `/g1/mobility/status` | String (JSON) | el árbitro de movilidad |
| `/cmd_vel` | Twist | sólo `mobility_authority.py` |
| `/g1/odom`, `/g1/joint_states` | Odometry, JointState | el robot |
| `/g1/head_cam/image` | Image | el robot |
| `/g1/head_cam/depth` | Image 32FC1 | el robot |
| `/g1/head_cam/camera_info` | CameraInfo | el robot |
| `/tf`: `map` → `head_cam_optical` | TransformStamped | el robot simulado |
| `/g1/lidar/points` | PointCloud2 | experimental; no usar hasta pasar `check_lidar.py` |

Cada pieza se puede reemplazar sin tocar las demás mientras respete su contrato:
la navegación por Nav2, RT-DETR por otro detector que publique las mismas
cajas, el modelo de lenguaje por otro planificador, el robot simulado por el
real.

El planificador remoto recibe la orden original y un catálogo cerrado. Cada
capacidad del catálogo explica en castellano qué hace, qué argumento acepta,
qué condiciones necesita y qué resultado deja. El modelo devuelve solamente
un plan JSON; no puede publicar movimiento. El servidor lo valida y la Jetson
lo vuelve a validar contra su propia copia antes de ejecutarlo. Si el servicio
remoto falla, la misión conocida usa el plan local de respaldo. Las
capacidades todavía pendientes aparecen como `placeholder` y bloquean la
ejecución honestamente al llegar a ellas.

## Los archivos

| Archivo | Qué hace |
|---|---|
| `g1_robot.py` | el robot: física, locomoción, brazos, cámara. Un proceso, lazo cerrado |
| `locomotion.py` | controladores intercambiables (NVIDIA AGILE / anterior / diagnóstico) |
| `arm_control.py` | control de brazos por poses con nombre |
| `perception.py` | la cámara de la cabeza y su publicación |
| `lidar.py` | integración RTX experimental, apagada por defecto |
| `g1_asset.py` | los cuerpos disponibles (12 y 29 articulaciones) y sus actuadores |
| `demo_scene.py` | la habitación, los objetos y el reloj digital |
| `skills/go_to.py` | navegación a un punto |
| `mobility_authority.py` | concede la movilidad a una sola fuente y alimenta `/cmd_vel` |
| `stand_hold.py` | mantiene una pose durante una espera; no navega |
| `skills/object_detector.py` | RT-DETR local con salida estándar de ROS 2 |
| `skills/open_vocabulary_detector.py` | manda un cuadro al detector remoto sólo por pedido |
| `skills/table_localizer.py` | une caja, profundidad y pose histórica; publica el punto en el mapa |
| `skills/detection_adapter.py` | conserva cuadros acotados, clasifica color y recorta el reloj |
| `mission_contract.py` | contrato de misión, pasos, estados y decisiones |
| `skill_catalog.py` | catálogo explicado y contratos que recibe el planificador |
| `model_trace.py` | contrato de trazabilidad de modelos, incluido el texto literal |
| `agent/agent.py` | valida y ejecuta localmente el plan; nunca entrega motores al LLM |
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

**El LLM propone; la Jetson decide si el plan es ejecutable.** El modelo ve
descripciones completas, no sólo nombres de capacidades, pero su salida sigue
siendo datos no confiables. Dos validaciones independientes rechazan skills
inventadas, argumentos desconocidos y pasos ordenados antes de cumplir sus
condiciones. El tablero conserva el pedido exacto, la respuesta literal y el
plan aceptado.

**Llegar tiene memoria.** El balanceo del bípedo hacía que la posición cruzara
el límite de 10 cm mientras terminaba de orientar el cuerpo, alternando para
siempre entre posición y ángulo. Como el verificador de objetivos de Nav2, la
navegación recuerda que ya alcanzó la posición y usa un margen de 10 cm
mientras cierra el ángulo. Las pruebas de mesa terminaron a 10,4 y 11,5 cm;
eso sirve como pose de observación, no como alineación de agarre.

**La pose de brazos cambia lo que ve la cámara.** En `reposo` las manos ocupan
las esquinas superiores. `listo` y `transporte` dejan libre todo el cuadro.
Las tres poses fueron verificadas con ángulos reales y con altura,
desplazamiento e inclinación del cuerpo; la búsqueda visual debe usar `listo`.

**Un dibujo de límites no es una habitación.** La escena anterior era un piso
abierto aunque el tablero mostrara un rectángulo. Ahora los mismos límites
crean cuatro paredes con colisión. Quietud (error p95 de 1 cm), caminata
(2,46 m) y frenado (2 cm) siguieron pasando con RTF 0,23.

**El LiDAR no está validado dentro del G1.** El ejemplo oficial aislado produjo
37.048 puntos, pero la integración en IsaacLab 5.1 siguió en cero incluso
frente a una pared. Montaje, cámara, frecuencia de render, visor, Fabric y el
puente ROS quedaron separados por experimentos. El detalle está en
[`LIDAR_STATUS.md`](LIDAR_STATUS.md); el flag `--lidar` no forma parte del
lanzamiento normal.

## Lo siguiente

La cámara, sus memorias acotadas y lo que muestra el tablero están explicados
en [`PERCEPTION_ARCHITECTURE.md`](PERCEPTION_ARCHITECTURE.md).

1. **Agregar el barrido visual activo** para buscar la mesa correcta sin pasarle una coordenada.
2. **Convertir la superficie encontrada en una pose segura de aproximación.**
3. **Guardar `home` y regresar** sin carga.
4. **Probar `transporte` caminando**: quieto ya es estable, pero falta medir rumbo;
   probar una postura más cercana a neutral y luego la escalera de cargas.
5. **Mapa y localización**: retomar el LiDAR sólo cuando su nube cruda pase el
   verificador; hasta entonces la profundidad de cámara mide objetos, no crea
   una falsa localización perfecta.
6. **Reloj simulado** (`/clock` + `use_sim_time`) para medir plazos en tiempo
   de simulación.
7. **El agarre** con un VLA entrenado por nosotros.
8. **Voz** para reemplazar la publicación manual de texto; el planificador
   semántico ya está conectado y acotado por el catálogo.
