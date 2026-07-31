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
| Carga en las manos | 0,5 kg aprobados en tres caminatas y tres navegaciones; 1 kg excede el margen del hombro derecho |
| Habitación física | funciona — cuatro paredes con colisión, alineadas con el tablero |
| Cámara de cabeza | funciona — color, profundidad y calibración sincronizados |
| LiDAR simulado | experimental; aislado funciona, integrado aún entrega nubes vacías |
| Detector local RT-DETR | integrado; umbral medido en la escena: mesa 6/6, pared 0/5 |
| Búsqueda visual abierta | integrada por pedido; roja y azul 3/3, red mala y corte probados |
| Mesa visual a punto del mapa | funciona — roja y azul caen sobre su superficie real |
| Lectura del reloj en servidor | funciona — 3/3 limpia y 3/3 con red mala |
| Navegación a un punto | funciona — Action cancelable `/g1/navigate_to_pose`, con progreso y regreso a `STAND` |
| Giro relativo | funciona — Action estándar `/g1/spin`, cancelable y con distancia angular |
| Barrido visual activo | funciona — cinco vistas superpuestas, confirmación remota sólo ante candidato |
| Corte del servidor | funciona — la misión falla explícitamente y el robot queda local |
| Preaproximación a la mesa | funciona — calcula pose desde profundidad, navega y vuelve a confirmar |
| Alineación fina a la mesa | funciona — `DockRobot`, visión continua, 3 cm y 2°; un estancamiento azul exigió un reintento |
| Ejecutor de misión | todos los pasos están conectados; falta repetir la misión integral y validarla visualmente |
| Transporte sin agarre | `attach_payload` con 0,5 kg físicos y visibles, validado sin afirmar agarre |
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
              | recorte/cuadro JPEG  /g1/navigate_to_pose /g1/arm_pose
              |                                  |
              +-- [ open_vocabulary_detector.py ] [ object_detector.py ]
                         |                 \             /
                 [ table_localizer ]        [ adapter ] [ go_to.py ]
                         |                             |
              /g1/table_detections_3d      /g1/cmd_vel/navigation
                         |                 /g1/cmd_vel/alignment
                  [ align_with_table ]             |
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
| `/g1/navigate_to_pose` | Action NavigateToPose | objetivo, progreso, resultado y cancelación de navegación |
| `/g1/spin` | Action Spin | giro relativo, progreso angular, resultado y cancelación |
| `/g1/dock_to_table` | Action DockRobot | alineación fina, progreso, reintentos y resultado |
| `/g1/navigation/goal` | PoseStamped | copia observable del objetivo para el tablero |
| `/g1/goal` | PoseStamped | compatibilidad temporal con verificadores anteriores |
| `/g1/nav_status` | String | relato temporal para verificadores anteriores |
| `/g1/object_detections` | Detection2DArray | el detector neuronal |
| `/g1/perception/search_request` | String (JSON) | agente o verificador |
| `/g1/open_vocabulary_detections` | Detection2DArray | búsqueda puntual |
| `/g1/table_detections_3d` | Detection3DArray | localizador de mesas |
| `/g1/object_detections_3d` | Detection3DArray | superficie visible del objeto transportable |
| `/g1/perception/search_status` | String (JSON) | búsqueda puntual |
| `/g1/detections` | String (JSON) | el adaptador de la demo |
| `/g1/clock_crop/compressed` | CompressedImage | el adaptador local |
| `/g1/perception/evidence/compressed` | CompressedImage | cuadro exacto enlazado a una detección |
| `/g1/model_input/compressed` | CompressedImage | agente o detector puntual; sólo la imagen realmente enviada al modelo |
| `/g1/arm_pose` | String | el agente |
| `/g1/payload_request` | String (JSON) | agente o verificador; agrega o retira carga |
| `/g1/payload_status` | String (JSON) | robot; masa aplicada y puntos físicos releídos |
| `/g1/alignment_status` | String (JSON) | mediciones y fase de la alineación fina |
| `/g1/cmd_vel/{stand,navigation,alignment,manual,test}` | Twist | cada fuente identificada |
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
| `navigation_core.py` | cálculo puro de movimiento y verificación de progreso |
| `execution_core.py` | vigilancia común de plazo y pérdida de respuesta |
| `skills/go_to.py` | servidor cancelable de navegación con contrato de Nav2 |
| `skills/align_with_table.py` | corrección visual fina con contrato `DockRobot` |
| `table_alignment_core.py` | control puro y tolerancias de alineación |
| `mobility_authority.py` | concede la movilidad a una sola fuente y alimenta `/cmd_vel` |
| `stand_hold.py` | mantiene una pose durante una espera; no navega |
| `skills/object_detector.py` | RT-DETR local con salida estándar de ROS 2 |
| `skills/open_vocabulary_detector.py` | manda un cuadro al detector remoto sólo por pedido |
| `skills/table_localizer.py` | une cajas, profundidad y pose histórica para mesas y objetos |
| `skills/detection_adapter.py` | conserva cuadros acotados, clasifica color y recorta el reloj |
| `mission_contract.py` | contrato de misión, pasos, estados y decisiones |
| `skill_catalog.py` | catálogo explicado y contratos que recibe el planificador |
| `model_trace.py` | contrato de trazabilidad de modelos, incluido el texto literal |
| `agent/agent.py` | valida y ejecuta localmente el plan; nunca entrega motores al LLM |
| `AGENT_EXECUTION_PLAN.md` | estado y criterios de la ejecución adaptable paso por paso |
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

**Preaproximarse no es alinearse para agarrar.** El flujo de Nav2 para
acercarse a infraestructura usa dos etapas: primero llega a una zona desde la
que todavía puede ver el objetivo; después vuelve a detectarlo y entra en un
control visual fino. Nuestra primera prueba intentó confirmar la mesa a 0,9 m:
una corrida quedó a 0,543 m y otras perdieron el mueble por debajo de la
cámara. La pose gruesa quedó por eso a 2,2 m del punto de superficie medido.
La validación integral la confirmó a 1,968 m, con 9,8 cm de error de
navegación, confianza 0,94 y cuerpo a 0,737 m. Desde allí,
`align_with_table` actualiza continuamente la mesa y corrige como máximo a
0,15 m/s. Rojo terminó a 1,6 cm y 1,58°; azul a 0,2 cm y 0,25°. Una primera
corrida azul se estancó y el reintento idéntico pasó, por lo que existe un
único reintento observable. Alinearse habilita la prueba de carga, no afirma
que las manos hayan agarrado.

**Una caja y profundidad no son una pose de agarre.** El objeto simulado es
un cilindro liso que RT-DETR reconoce de forma estable como `cup`, no como
`bottle`. El contrato acepta ambos como `transport_object`, mide su superficie
visible y verifica que esté sobre la mesa elegida. La posición y orientación
completas seguirán el flujo de NVIDIA con FoundationPose o la policy/VLA de
agarre; la etapa actual no inventa esos datos faltantes. Tres mediciones 3D
quedaron a 3,2 cm en horizontal y 5–6 mm en altura de la referencia física;
el control del detector dio 6/6 positivos y 0/5 falsos.

**Carga simulada no significa agarre.** `attach_payload` puede agregar masa
física verificada a las dos muñecas después de la alineación y mostrar un
bulto naranja entre ellas. Esto permite medir quietud, caminata, frenado y
regreso cargado antes de tener Dex3. El tablero lo marca como “agarre NO
validado”; el objeto visible no tiene una segunda masa ni colisión. Es una
adaptación temporal, no el flujo oficial de manipulación de NVIDIA.

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
condiciones. Después de cada resultado, el modelo puede continuar, pedir un
único reintento, modificar sólo lo pendiente, pedir ayuda, completar con éxito
o detenerse. `complete` sólo es válido si el último paso pasó y no queda
ninguno pendiente. El tablero conserva el plan inicial, la revisión, el pedido
exacto, la respuesta literal y el resultado aceptado.

**Llegar tiene memoria.** El balanceo del bípedo hacía que la posición cruzara
el límite de 10 cm mientras terminaba de orientar el cuerpo, alternando para
siempre entre posición y ángulo. Como el verificador de objetivos de Nav2, la
navegación recuerda que ya alcanzó la posición y usa un margen de 10 cm
mientras cierra el ángulo. Las pruebas de mesa terminaron a 10,4 y 11,5 cm;
eso sirve como pose de observación, no como alineación de agarre.

**Buscar no es girar a ciegas ni ejecutar el modelo caro todo el tiempo.**
La cámara cubre 108,1° horizontalmente. Cinco vistas separadas 72° dejan
36,1° de superposición y cubren la vuelta completa. En cada vista el detector
local y una señal amplia de color sólo proponen candidatos; Grounding DINO,
el color dentro de su caja y la profundidad son quienes confirman y ubican la
mesa. En dos corridas completas la mesa roja apareció en la cuarta vista y
sólo se hizo una llamada remota por corrida. El primer giro acumuló 17,1 cm de
desplazamiento: sirve en la habitación abierta, pero no para alinear un agarre.

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

1. **Ejecutar la misión integral**: `find_object`, alineación, 0,5 kg y regreso
   ya pasaron por partes; falta observarlos y medirlos en una misma corrida.
2. **Validar visualmente con Lucas** la separación final, el bulto entre las
   manos y el regreso cargado. Los números no sustituyen esa inspección.
3. **Medir la confiabilidad de la alineación**: una de tres aproximaciones
   largas se estancó y pasó en el primer reintento idéntico.
4. **Migrar el barrido completo a una Action cancelable**, conservando el
   giro `Spin` actual y evitando que una cancelación llegue sólo entre vistas.
5. **Continuar la escalera de cargas** solamente si cada nivel pasa quietud,
   caminata y frenado; probar una postura más cercana a neutral si falla.
6. **Mapa y localización**: retomar el LiDAR sólo cuando su nube cruda pase el
   verificador; hasta entonces la profundidad de cámara mide objetos, no crea
   una falsa localización perfecta.
7. **Reloj simulado** (`/clock` + `use_sim_time`) para medir plazos en tiempo
   de simulación.
8. **El agarre** con un VLA entrenado por nosotros.
9. **Voz** para reemplazar la publicación manual de texto; el planificador
   semántico ya está conectado y acotado por el catálogo.
