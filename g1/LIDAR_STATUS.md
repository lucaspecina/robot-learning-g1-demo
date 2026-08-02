# Estado del LiDAR simulado

Fecha de corte: 2026-08-02.

## Para qué lo queremos

El LiDAR mide distancias alrededor del robot con rayos láser. Su tarea será
construir el mapa geométrico de la habitación, ubicar al robot dentro de ese
mapa y alimentar la evitación de obstáculos. La cámara sigue teniendo otra
tarea: reconocer qué objeto es una mesa, su color y la superficie exacta a la
que acercarse.

El contrato transferible elegido es el estándar de ROS 2:
`sensor_msgs/PointCloud2` en `/g1/lidar/points`, con marco `lidar_link`.
El perfil `Example_Rotary` sólo sirve para comprobar la integración. No
representa el sensor físico del G1: la página pública de Unitree informa
“3D LiDAR”, pero no identifica el modelo ni su montaje para todas las
configuraciones EDU.

## Resultado vigente

La puerta de aceptación del lanzamiento normal pasó con estas mediciones:

| Salida o conducta | Tres muestras / repeticiones |
|---|---:|
| Nube 3D acumulada | 285.234–285.353 puntos, 359,9°, 3,34 MiB por vuelta |
| Barrido plano `/scan` | 1.066 rayos, 762 válidos, 359,4°, 0,67 Hz de pared |
| Mapa quieto de SLAM Toolbox | 138 × 148 celdas de 5 cm; 117 ocupadas |
| Mapa local Nav2 | 60 × 60 celdas; 242 letales y 258 de margen |
| Quietud después de integrar seguridad | 60 s, error final y máximo 4 cm |
| Caminata y frenado | 2,14 m, 17 cm lateral, freno en 4 cm, de pie |
| Aproximación a una pared | 0,50 / 0,51 / 0,51 m; avance extra 2–3 cm |
| Cajón bajo de 45 cm, LiDAR 3D provisional | **0 puntos** dentro del cajón |
| Mismo cajón, cámara de profundidad | **13.114–13.440 puntos** dentro del cajón |

El mapa local ya alimenta al Nav2 completo. NavFn calcula la ruta global y DWB
elige avance y giro mirando obstáculos actuales; Collision Monitor conserva la
última palabra antes de `/cmd_vel`. El mapa fijo no contiene el cajón: su celda
vale `0`, mientras el mapa vivo de Nav2 midió `99` en 3/3 repeticiones. El
LiDAR sí detecta el obstáculo que apareció después de mapear.

El cuerpo dejó 56–60 cm alrededor del cajón y nunca cayó. Dos acciones
terminaron a 17,0 y 19,1 cm físicos; una tercera agotó el plazo lejos del
objetivo. Esquive y seguridad funcionan, pero la navegación completa y la
localización móvil todavía no son repetibles.

El perfil 3D provisional de NVIDIA no resuelve obstáculos bajos: sus 128
emisores cubren sólo de -15° a +10°. Desde la cabeza, el rayo inferior pasa
cerca de un metro por encima de un cajón de 45 cm ubicado a 1,8 m. La nube
confirmó la geometría con cero puntos sobre su planta. Agregar esa nube a Nav2
no mejoró la métrica y se revirtió.

La cámara RGB-D sí resolvió el caso por el flujo estándar. El componente
oficial `depth_image_proc` de ROS convierte `/g1/head_cam/depth` más su
calibración en `/g1/head_cam/points`; Nav2 recibe ese `PointCloud2` además del
LaserScan. Detectó entre 13.114 y 13.440 puntos del cajón y el mapa vivo pasó
de libre `0` a ocupado `100`, mientras el mapa fijo siguió libre. La
transformación de la cámara ya no es `map -> cámara` calculada con la posición
perfecta de Isaac: ahora nace en `base_link` y sólo describe el montaje físico,
como hará el URDF con los encoders del G1 real.

Se corrigió además una causa estructural: en Isaac 5.1,
`LidarRtx.get_world_pose()` dejó de seguir al padre móvil y ubicaba falsamente
el sensor 56 cm por delante. Ahora se mide una vez el montaje fijo respecto de
`torso_link` y se reconstruye su pose desde el cuerpo físico en cada estado.
Es el equivalente simulado del árbol de coordenadas que publicará el URDF del
G1 real; no consulta la posición de una pared ni de un objetivo.

## Historial de diagnóstico

Las cifras siguientes explican cómo se aisló la falla inicial. Las mediciones
vigentes de aceptación son las de la sección anterior.

| Prueba, cambiando una variable | Resultado |
|---|---:|
| API oficial `LidarRtx`, sensor quieto a 1 m y pared a 3 m | **37.048 puntos**, después de 5 cuadros vacíos |
| Integración inicial en G1, publicador oficial ROS | 0 puntos; 129 mensajes vacíos |
| Dibujar cada paso en vez de uno cada 25 | 0; 408 vacíos |
| Clase completa `LidarRtx` en vez del comando de bajo nivel | 0; 128 vacíos |
| Sensor fijo en el mundo en vez de montado en la cabeza | 0; 128 vacíos |
| Sin la cámara RGB-D | 0; 155 vacíos |
| Sin visor remoto | 0; 174 vacíos |
| Con una pared física grande a 3 m | 0; 175 vacíos |
| Optimización Fabric apagada | 0; 178 vacíos |
| Lectura interna, antes del puente ROS | 0 sostenido |
| Aplicación completa de Isaac Sim | prueba inválida: el puente ROS se cayó al iniciar |
| Misma referencia mínima con `AppLauncher`, sin extensión RTX explícita | falla al importar `isaacsim.sensors.rtx` |
| `AppLauncher` con `isaacsim.sensors.rtx` habilitada | **72.841 puntos**, después de 1 cuadro vacío |
| `SimulationContext` de la demo, 500 Hz y render a 20 Hz | **9.216 puntos**, después de 1 cuadro vacío |
| Caso anterior + G1 + habitación + puente ROS | **9.216 puntos**, después de 13 cuadros vacíos |
| LiDAR a 0,12 m o 0,25 m de `head_link` | **0 puntos** en 20 cuadros |
| Mismo montaje a 0,35 m de `head_link` | **9.216 puntos**, después de 1 cuadro vacío |
| G1 integrado: tres nubes ROS completas | **10.008–23.040 puntos**, 6,8 Hz de pared, 0 vacías |
| Quietud integrada con LiDAR | **pasa**: error final 2 cm, máximo 6 cm |
| Caminata y frenado integrados con LiDAR | **pasa**: 3,22 m, frena en 2–5 cm, sigue de pie |
| 40 nubes quieto contra paredes conocidas | mediana **6,0 cm**, p95 **6,3 cm**, peor 6,8 cm |
| 40 nubes caminando, ventana 1 | mediana **6,5 cm**, p95 **8,5 cm**, peor 14,7 cm |
| 40 nubes caminando, ventana 2 | mediana **8,6 cm**, p95 **12,2 cm**, peor 17,2 cm |
| 20 nubes durante frenado/final | mediana **4,3 cm**, p95 **7,5 cm**, peor 13,7 cm |

La ausencia de paredes era un defecto real de la escena anterior, pero **no**
era la causa de la nube vacía: una pared aislada no cambió el resultado. La
lectura interna en cero también descarta que ROS estuviera perdiendo puntos ya
calculados.

El caso mínimo con `AppLauncher` encontró primero una dependencia que antes
quedaba oculta: la experiencia liviana de Isaac Lab no carga la extensión
`isaacsim.sensors.rtx`. Después se reprodujo, una variable por vez, el paso de
500 Hz, el render a 20 Hz, el cuerpo oficial, la habitación y el puente ROS.
Todos conservaron puntos.

La falla apareció recién al montar el sensor dentro de la cabeza. El perfil
provisional barre 360° y la carcasa visual del USD es opaca a sus rayos: a
0,12 m y 0,25 m del origen de `head_link` la nube fue exactamente vacía. A
0,35 m, ya fuera de la carcasa, recuperó los 9.216 puntos del caso fijo. Ese
montaje es funcional pero **no se presenta como réplica del G1 físico**: debe
reemplazarse cuando Unitree confirme el modelo y la posición exactos del
LiDAR comprado.

La integración completa ya pasa la puerta ROS: tres nubes distintas, finitas
y no vacías, con entre 10.008 y 23.040 puntos según la parte del barrido.
También pasó quietud, caminata y frenado con el costo del sensor activo.

Motion BVH se solicita sólo con LiDAR porque NVIDIA lo exige para compensar
el movimiento del barrido. El ajuste global queda activo, pero el plugin del
sensor todavía avisa que no puede aplicarlo. La VM usa una Tesla T4 de
arquitectura Turing y capacidad 7.5; un ingeniero de NVIDIA indica que esta
función requiere Ampere o posterior. Por lo tanto, **la nube quieta está
validada y la nube móvil tiene una deformación medida**. Al caminar a la
velocidad de prueba, el p95 creció entre 2,2 y 5,9 cm sobre la referencia
quieta, con picos por cuadro de hasta 17,2 cm. La medición incluye también el
desfase real entre `/g1/odom` y la nube, que es justamente lo que recibirá el
mapa actual.

La decisión es usar esta salida para el primer mapa y la navegación gruesa de
la habitación, donde un error transitorio de centímetros es tolerable. No se
usa para los últimos centímetros frente a una mesa ni para agarrar: ahí se
mantiene la cámara con profundidad. Si el mapa real muestra paredes dobles o
inestables, no se ajustan filtros a ciegas: se cambia la VM a una GPU Ampere o
posterior, o se evalúa el LiDAR PhysX oficial como experimento separado.

## Decisión

- El lanzamiento normal habilita el LiDAR y levanta SLAM Toolbox en la Jetson.
- La cadena validada es
  `map → odom → base_footprint → base_link → lidar_link`. La huella plana es
  virtual: navegación no debe interpretar la altura ni el balanceo de la
  pelvis como altura o pendiente del piso.
- `/clock` publica el tiempo de física. Con los dos perfiles LiDAR activos, la
  medición fue RTF 0,32–0,33; los plazos de navegación ya no deben depender de
  cuánto tarda Azure en simular un segundo.
- La nube 3D se acumula hasta completar 359,9° en `/g1/lidar/points`. No se usa
  como barrera inmediata porque, con RTF 0,23 en esta T4, cada vuelta pesa
  3,34 MiB y tarda demasiado en tiempo de pared. Para SLAM y seguridad, Isaac
  5.1 necesita un perfil 2D separado y publica la vuelta en `/scan_raw`.
  `navigation/laser_scan_adapter.py` corrige sólo una inconsistencia medida del
  ejemplo NVIDIA: declara 1.067 rayos y entrega 1.066. No altera distancias.
- `/scan` pasó tres vueltas de 1.066 rayos, 359,4°, 762 retornos válidos y
  0,67 Hz de pared. Su alcance cercano se ajustó a 5 cm, valor publicado para
  el Unitree L2: el perfil de ejemplo de Isaac ocultaba la pared por debajo de
  1 m y volvía inútil la zona de parada. Un perfil NVIDIA/SLAMTEC alternativo
  produjo datos más viejos, falló la distancia de frenado y se revirtió.
- Quieto, SLAM Toolbox produjo un mapa de 138 × 148 celdas a 5 cm, con 7.958
  libres y 121 ocupadas. `map → odom` quedó en identidad, como debe ocurrir
  contra la odometría exacta actual de Isaac.
- La ida y vuelta física anterior pasó dos veces. En la final se alejó 1,91 m,
  llegó con 10 cm de error y volvió con 8,6 cm, de pie a 0,77 m.
- El mapa móvil **no pasa todavía la puerta de precisión**: después de esa ida
  y vuelta contradijo la referencia exacta de Isaac en 0,26 m y 8,4°. La T4 no
  soporta Motion BVH y el barrido móvil queda deformado. No se oculta ajustando
  tolerancias de SLAM.
- El mapa fijo + AMCL evitó que el mapa se deforme, pero no corrigió el sensor.
  Con el mapa limpio, dos corridas útiles terminaron con 0,27–0,28 m de
  corrección y 1,3–3,2°; la tercera se bloqueó aun con sólo 0,04 m / 3,3°.
  Limitar la navegación a 0,15 m/s dejó la corrección en 0,04 m / 0,7°, pero la
  policy no produjo desplazamiento útil; el límite se revirtió.
- La validación final del bloque confirmó que las piezas están separadas: el
  robot quieto quedó dentro de 4 cm; caminó 2,11 m, se desvió 12 cm y frenó en
  8 cm. Locomoción pasa, Nav2 rodea y localización móvil falla.
- El LiDAR PhysX oficial se probó como diagnóstico, no como reemplazo del
  sensor real. En una pared conocida, quieto dio error 0 y sobre un soporte a
  0,30 m/s dio mediana, p95 y máximo de 5 mm; conserva
  `tools/check_physx_lidar_standalone.py` para repetirlo. Integrado al G1, en
  cambio, el plano de 360° interceptó el propio cuerpo: 343 rayos quedaron
  clavados en el mínimo de 5 cm y Collision Monitor anuló la caminata. Un
  filtro de 10 cm y elevar el montaje 10 cm no resolvieron la falsa emergencia
  durante el balanceo. Se revirtieron ambos: no se deja un sensor que sólo
  funciona ocultando una parte creciente del espacio cercano.
- La versión Jazzy instalada expone para la fuente `scan` sólo `topic`,
  `source_timeout`, `type` y `enabled`; no tiene todavía las zonas geométricas
  de exclusión que Nav2 actual recomienda para quitar retornos del propio
  robot. El sensor físico deberá entregar una nube filtrable por cuerpo y
  altura, no un plano simulado flotante.
- Se conserva `tools/check_lidar_standalone.py` como referencia positiva y
  `tools/check_lidar.py` como puerta obligatoria para la integración.
- `tools/check_time_and_tf.py`, `tools/check_laser_scan.py` y
  `tools/check_slam_map.py` verifican tiempo, coordenadas, vuelta completa y
  calidad del mapa. La última falla si SLAM corrige más de 15 cm o 5° mientras
  usemos la referencia perfecta de Isaac.
- `tools/measure_lidar_wall_error.py` compara cada nube con las cuatro paredes
  físicas y deja una métrica repetible para detectar regresiones en movimiento.
- `tools/check_local_costmap.py` exige resolución, tamaño, datos frescos,
  obstáculos, margen inflado, centro libre y coincidencia con la pose actual.
- `tools/check_collision_safety.py` prueba la propiedad exclusiva de los
  topics y tres detenciones físicas, no sólo que existan procesos.
- La distancia a las mesas continúa usando RGB-D: imagen de color más
  profundidad sincronizada. Eso existe en el robot real si la cámara elegida
  entrega profundidad, pero debe confirmarse el modelo exacto antes de comprar.
- La misma profundidad alimenta obstáculos bajos mediante el paquete oficial
  `depth_image_proc`. Se conserva ObstacleLayer, recomendado por Nav2 cuando el
  presupuesto es limitado; VoxelLayer queda como mejora medible, no como
  requisito inventado.
- Collision Monitor conserva sólo `/scan`. La cámara ve 20.054 puntos del
  propio torso entre 17 y 23 cm delante del marco del cuerpo y produjo una
  falsa parada inmediata. El paquete comunitario `robot_body_filter` implementa
  el filtrado correcto contra URDF, pero no está publicado para Jazzy y este
  banco aún no publica todos los eslabones del G1. No se reemplazó por un radio
  ciego que también ocultaría obstáculos reales cercanos.
- La nube se transforma exactamente a `odom`. Hacia `map`, AMCL puede quedar
  varios segundos simulados atrás cuando el robot está quieto; el mapa local
  sigue recibiendo todos los cuadros y el global los recibe cuando la
  localización está fresca.
- La pose perfecta que Isaac conoce no se presentará como localización real.

## Próximo bloque útil

La integración, el mapa quieto, el mapa local, la barrera final y la detección
de obstáculos bajos están cerrados. La localización continua en movimiento y
el cierre repetible de las acciones Nav2, no.
El siguiente bloque debe seguir el orden estándar:

1. repetir la misma ida y vuelta en una VM Ampere o posterior y exigir menos
   de 15 cm / 5° de corrección;
2. no integrar el LiDAR PhysX 2D al G1: ya aisló el error RTX, pero falló la
   prueba de cuerpo propio y fue retirado del camino normal;
3. confirmar el sensor real antes de fijar perfil, frecuencia y montaje; la
   cámara de profundidad ya cubre el hueco bajo sin fingir otro LiDAR;
4. corregir la localización móvil: con el cajón bajo el cuerpo dejó 42,5 cm,
   no cayó y terminó físicamente a 2,8 cm, pero Nav2 agotó el plazo porque AMCL
   acumuló 41 cm de corrección;
5. repetir el rodeo físico ya medido y exigir llegada menor a 15 cm, pero no
   declarar despliegue transferible hasta aprobar la localización;
6. mantener Collision Monitor como único publicador final de `/cmd_vel`.

Isaac Sim 5.1 ya figura como versión sin soporte. Una prueba posterior sobre
una versión nueva queda justificada si la integración continúa fallando, pero
siempre en una rama o instalación separada: una actualización no puede poner
en riesgo la locomoción AGILE ya verificada.

Antes de permitir que Nav2 mueva el robot:

1. confirmar con Unitree el modelo, orientación y frecuencia del LiDAR real;
2. usar un perfil que reproduzca ese patrón, no un archivo comunitario
   renombrado;
3. publicar y comprobar la transformación `base_link` → `lidar_link`;
4. medir frecuencia, ancho de banda y costo en RTF con `/scan`;
5. reemplazar la pose perfecta de Isaac por una estimación realizable;
6. no usar todavía el mapa móvil para afirmar navegación Nav2 autónoma.

## Fuentes primarias y confiables

- [NVIDIA: tutorial ROS 2 para RTX LiDAR](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_rtx_lidar.html)
- [Nav2: transformaciones requeridas](https://docs.nav2.org/setup_guides/transformation/setup_transforms.html)
- [Nav2: huella virtual proyectada al piso](https://docs.nav2.org/setup_guides/urdf/setup_urdf.html)
- [SLAM Toolbox: configuración online asíncrona](https://github.com/SteveMacenski/slam_toolbox/blob/ros2/config/mapper_params_online_async.yaml)
- [NVIDIA: fallas de LaserScan que rompen SLAM Toolbox](https://forums.developer.nvidia.com/t/ros2-laserscan-faulty-data/231738)
- [NVIDIA: extensión RTX y Motion BVH](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/sensors/isaacsim_sensors_rtx.html)
- [NVIDIA: LiDAR PhysX y disparo simultáneo a 0 Hz](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/sensors/isaacsim_sensors_physx_lidar.html)
- [NVIDIA: requisito Ampere o posterior informado por su equipo](https://forums.developer.nvidia.com/t/motionbvh-for-lidar-model-not-enabled/297482)
- [NVIDIA: la Tesla T4 pertenece a la arquitectura Turing](https://www.nvidia.com/content/dam/en-zz/Solutions/design-visualization/technologies/turing-architecture/NVIDIA-Turing-Architecture-Whitepaper.pdf)
- [NVIDIA Isaac Sim 5.1: notas de versión](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/overview/release_notes.html)
- [Unitree G1: página oficial del producto](https://www.unitree.com/g1)
- [Unitree LiDAR SDK: salida PointCloud2 oficial](https://github.com/unitreerobotics/unilidar_sdk2)
- [Unitree Point-LIO: referencia con LiDAR L2 e IMU](https://github.com/unitreerobotics/point_lio_unilidar)
- [Nav2: mapa local, capa de obstáculos e inflación](https://docs.nav2.org/setup_guides/sensors/mapping_localization.html)
- [Nav2: configuración de Collision Monitor](https://docs.nav2.org/configuration/packages/collision_monitor/configuring-collision-monitor-node.html)
- [ROS 2 Jazzy: conversión oficial de profundidad a nube](https://docs.ros.org/en/jazzy/p/depth_image_proc/doc/index.html)
- [Nav2: LaserScan y PointCloud2 combinados](https://docs.nav2.org/setup_guides/sensors/mapping_localization.html)
- [ROS Index: robot_body_filter](https://index.ros.org/p/robot_body_filter/)
- [IsaacLab: caso comunitario de LiDAR vacío y Fabric](https://github.com/isaac-sim/IsaacLab/discussions/2082)
