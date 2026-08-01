# Estado del LiDAR simulado

Fecha de corte: 2026-08-01.

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

## Resultado medido

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

La ausencia de paredes era un defecto real de la escena anterior, pero **no**
era la causa de la nube vacía: una pared aislada no cambió el resultado. La
lectura interna en cero también descarta que ROS estuviera perdiendo puntos ya
calculados.

El caso mínimo con `AppLauncher` encontró una dependencia que antes quedaba
oculta: la experiencia liviana de Isaac Lab no carga la extensión
`isaacsim.sensors.rtx`. La documentación oficial exige habilitarla antes de
usar el sensor. Al hacerlo, la misma pared produjo `72.841` puntos. Esto
demuestra que `AppLauncher` no es incompatible con el LiDAR y deja un cambio
concreto para la integración del G1: habilitar la extensión antes de importar
`LidarRtx` y volver a ejecutar la puerta ROS completa.

## Decisión

- El lanzamiento normal no habilita `--lidar`.
- No se conecta una nube vacía a SLAM ni a Nav2. SLAM significa construir un
  mapa y ubicarse dentro de él mientras el robot se mueve.
- Se conserva `tools/check_lidar_standalone.py` como referencia positiva y
  `tools/check_lidar.py` como puerta obligatoria para la integración.
- La distancia a las mesas continúa usando RGB-D: imagen de color más
  profundidad sincronizada. Eso existe en el robot real si la cámara elegida
  entrega profundidad, pero debe confirmarse el modelo exacto antes de comprar.
- La pose perfecta que Isaac conoce no se presentará como localización real.

## Próximo experimento útil

Ejecutar el G1 quieto con una sola diferencia respecto de la integración que
publicaba vacío: `isaacsim.sensors.rtx` habilitada explícitamente. La prueba
debe pasar `tools/check_lidar.py`, que exige tres nubes completas, puntos
finitos, horas distintas y el marco `lidar_link`. Si pasa, habilitar Motion BVH
—la representación de movimiento que NVIDIA exige para corregir barridos de
un sensor móvil— y recién después repetir quietud, caminata y frenado.

Isaac Sim 5.1 ya figura como versión sin soporte. Una prueba posterior sobre
una versión nueva queda justificada si la integración continúa fallando, pero
siempre en una rama o instalación separada: una actualización no puede poner
en riesgo la locomoción AGILE ya verificada.

Después de recuperar puntos crudos:

1. confirmar con Unitree el modelo, orientación y frecuencia del LiDAR real;
2. usar un perfil que reproduzca ese patrón, no un archivo comunitario
   renombrado;
3. publicar la transformación `base_link` → `lidar_link`;
4. medir frecuencia, ancho de banda y costo en RTF;
5. repetir quietud, caminata y frenado;
6. recién entonces integrar mapa y localización.

## Fuentes primarias y confiables

- [NVIDIA: tutorial ROS 2 para RTX LiDAR](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_rtx_lidar.html)
- [NVIDIA: extensión RTX y Motion BVH](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/sensors/isaacsim_sensors_rtx.html)
- [NVIDIA Isaac Sim 5.1: notas de versión](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/overview/release_notes.html)
- [Unitree G1: página oficial del producto](https://www.unitree.com/g1)
- [Unitree LiDAR SDK: salida PointCloud2 oficial](https://github.com/unitreerobotics/unilidar_sdk2)
- [IsaacLab: caso comunitario de LiDAR vacío y Fabric](https://github.com/isaac-sim/IsaacLab/discussions/2082)
