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

- El lanzamiento normal todavía no habilita `--lidar`: primero se incorpora
  el árbol de coordenadas y se mide el costo completo del mapa.
- La nube ya puede alimentar el siguiente bloque. SLAM significa construir un
  mapa y ubicarse dentro de él mientras el robot se mueve.
- Se conserva `tools/check_lidar_standalone.py` como referencia positiva y
  `tools/check_lidar.py` como puerta obligatoria para la integración.
- `tools/measure_lidar_wall_error.py` compara cada nube con las cuatro paredes
  físicas y deja una métrica repetible para detectar regresiones en movimiento.
- La distancia a las mesas continúa usando RGB-D: imagen de color más
  profundidad sincronizada. Eso existe en el robot real si la cámara elegida
  entrega profundidad, pero debe confirmarse el modelo exacto antes de comprar.
- La pose perfecta que Isaac conoce no se presentará como localización real.

## Próximo bloque útil

Publicar y verificar la cadena estándar de coordenadas
`map → odom → base_link → lidar_link`, y convertir la nube 3D a un corte 2D
`/scan` para SLAM Toolbox y Nav2. El corte debe conservar obstáculos bajos
sin confundir piso, techo, brazos ni carga. La nube 3D se conserva para
percepción y obstáculos elevados.

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
6. recién entonces integrar mapa, localización y Nav2.

## Fuentes primarias y confiables

- [NVIDIA: tutorial ROS 2 para RTX LiDAR](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_rtx_lidar.html)
- [NVIDIA: extensión RTX y Motion BVH](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/sensors/isaacsim_sensors_rtx.html)
- [NVIDIA: requisito Ampere o posterior informado por su equipo](https://forums.developer.nvidia.com/t/motionbvh-for-lidar-model-not-enabled/297482)
- [NVIDIA: la Tesla T4 pertenece a la arquitectura Turing](https://www.nvidia.com/content/dam/en-zz/Solutions/design-visualization/technologies/turing-architecture/NVIDIA-Turing-Architecture-Whitepaper.pdf)
- [NVIDIA Isaac Sim 5.1: notas de versión](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/overview/release_notes.html)
- [Unitree G1: página oficial del producto](https://www.unitree.com/g1)
- [Unitree LiDAR SDK: salida PointCloud2 oficial](https://github.com/unitreerobotics/unilidar_sdk2)
- [IsaacLab: caso comunitario de LiDAR vacío y Fabric](https://github.com/isaac-sim/IsaacLab/discussions/2082)
