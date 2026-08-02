# Estado de la navegación Nav2

Fecha de corte: 2-ago-2026.

## Qué quedó integrado

La misión conserva `/g1/navigate_to_pose` y `/g1/spin`. `nav2_adapter.py`
adquiere la autoridad exclusiva, reenvía la acción al Nav2 oficial y devuelve
el mando a `STAND` en éxito, cancelación o falla. Nunca publica velocidad.

Nav2 usa:

- NavFn para calcular la ruta sobre el mapa global;
- DWB para seguirla y evitar obstáculos sobre el mapa local;
- Velocity Smoother para limitar cambios bruscos;
- Behavior Server para giro, retroceso y espera;
- BT Navigator para coordinar ruta, seguimiento y recuperaciones;
- dos costmaps de 5 cm y Collision Monitor como barrera final.

Los dos costmaps combinan ahora el corte horizontal `/scan` con
`/g1/head_cam/points`, una nube construida por `depth_image_proc`, el paquete
oficial de ROS para cámaras de profundidad. El primero cubre paredes alrededor;
la segunda cubre objetos bajos dentro del campo visual frontal.

La operación normal ya no modifica el mapa mientras camina. Sigue el flujo
recomendado por Nav2 para una habitación conocida: `map_server` carga el mapa
guardado y AMCL compara cada barrido con él para publicar `map -> odom`.
SLAM Toolbox queda como modo separado para construir o renovar el mapa. Los
dos modos se excluyen por proceso; nunca publican esa corrección a la vez.

La adaptación propia es el límite medido del G1: 0,30 m/s y 0,50 rad/s como
máximos, 0,10 m/s como velocidad útil mínima y huella cuadrada de 60 cm.

## Experimentos que decidieron el controlador

| Cambio único | Resultado |
|---|---|
| MPPI, 2.000 muestras | dos CPU saturadas; 1 m no llegó en 180 s de pared |
| MPPI, 1.000 muestras | siguió saturado; orden de 0,01–0,02 m/s, menor que la deriva |
| crítico oficial de velocidad muerta a 0,10 m/s | orden de ~0,05 m/s; no llegó |
| MPPI usando la orden anterior | ~0,06 m/s; no llegó |
| DWB con mínimo duro de 0,10 m/s | llegó a 0,50 m y volvió a `STAND` |
| espera interna 20 ms | dos éxitos y un aborto al aceptar una ruta bajo carga |
| espera interna 500 ms | repetición aprobada; no cambia movimiento ni seguridad |

La elección de DWB no afirma que sea universalmente mejor que MPPI. Es el
controlador oficial que cumple el presupuesto medido de esta Jetson simulada y
permite representar la velocidad mínima que esta policy realmente ejecuta.

Después de integrarlo al lanzador normal pasaron:

- quietud: 60 s, máximo 4 cm y final 3 cm;
- caminata directa: 2,16 m, desvío 4,6°, freno 0 cm, sin caída;
- sensor y mapas: vuelta de 359,4°, mapa global utilizable y obstáculo más
  cercano a 1,08 m con 35 cm libres alrededor del cuerpo.

La puerta de rodeo ya separa mapa permanente de obstáculo actual. El mapa fijo
tiene valor libre `0` en el centro del cajón; el mapa vivo de Nav2 midió `99`
en las tres repeticiones. Por lo tanto, el rodeo demuestra detección por LiDAR
y no conocimiento grabado de la escena.

En tres corridas el cuerpo dejó 0,56–0,60 m y la altura nunca bajó de 0,735 m.
Dos acciones terminaron, pero con 0,170 m y 0,191 m de error físico; la tercera
agotó 240 s a 2,836 m del objetivo. Las tres devolvieron el mando a `STAND`.
No se aprueba todavía la navegación completa ni su repetibilidad.

## Experimentos de localización móvil

| Cambio único | Resultado |
|---|---|
| SLAM online durante el rodeo | esquivó; corrección falsa de 0,93 m / 9,0° |
| mapa fijo + AMCL diferencial | corrección de 0,68 m / 11,8°; no llegó físicamente |
| AMCL omnidireccional, mapa que todavía contenía el cajón | mejoró a 0,42 m / 8,0°; esa corrida no prueba detección viva |
| mismo caso limitado a 0,15 m/s | localización 0,04 m / 0,7°, pero la policy casi no avanzó |
| mapa limpio + cajón sólo en el sensor, tres repeticiones | detección 3/3; dos llegadas a 17,0/19,1 cm y un bloqueo |
| cajón de 45 cm + LiDAR provisional | 0 puntos; mapa vivo quedó libre `0` |
| mismo cajón + profundidad estándar | 13.114–13.440 puntos; mapa vivo ocupado `100` |
| rodeo del cajón bajo | margen real 42,5 cm, altura mínima 0,730 m, error físico 2,8 cm; acción agotó plazo por 41 cm de error de localización |

La velocidad lenta no sirve como arreglo: cayó en la zona de órdenes que
mueven las piernas sin desplazar útilmente el cuerpo y fue revertida. El
modelo omnidireccional sí se conserva porque el contrato de la policy acepta
movimiento lateral y la deriva lateral está medida.

## Lo que todavía no está aprobado

- terminar físicamente a 15 cm en tres repeticiones después de rodear;
- repetición e inspección visual del rodeo completo;
- localización móvil transferible: Isaac todavía entrega odometría perfecta y
  la T4 no compensa bien el movimiento del barrido RTX;
- huella dinámica cuando brazos y carga sobresalen del torso.

No se seguirá afinando tolerancias de Nav2 para ocultar ese error. El próximo
experimento útil para la simulación es repetir con GPU Ampere o posterior. El
LiDAR PhysX oficial ya aisló la deformación en una pared móvil con 5 mm de
error, pero su plano 2D interceptó el propio G1 durante la marcha y activó la
parada de emergencia; filtros de 10 cm y un montaje 10 cm más alto no lo
resolvieron y la integración fue retirada. Para el robot real, el camino de
referencia es LiDAR + IMU con Point-LIO de Unitree y filtrado del propio cuerpo.

`tools/check_obstacle_navigation.py` rechaza la prueba antes de arrancar si el
cajón ya figura en el mapa fijo o si el mapa vivo no lo agregó. Luego mide
ruta, trayectoria física, distancia al obstáculo, altura, llegada, corrección
de localización y devolución de autoridad incluso cuando hay cancelación.
Con `--sensing-only` comprueba sólo la percepción sin autorizar movimiento.
`tools/check_depth_safety_zone.py` conserva la prueba negativa de la barrera:
midió 20.054 retornos del propio torso dentro de la zona de emergencia. Por
eso Collision Monitor sigue usando el LaserScan hasta disponer de filtrado
corporal basado en el URDF completo.

## Referencias oficiales

- [Nav2: selección de algoritmos](https://docs.nav2.org/setup_guides/algorithm/select_algorithm.html)
- [Nav2: DWB](https://docs.nav2.org/configuration/packages/configuring-dwb-controller.html)
- [Nav2: mapa local y controlador](https://docs.nav2.org/configuration/packages/configuring-controller-server.html)
- [Nav2: BT Navigator y sus plazos](https://docs.nav2.org/configuration/packages/configuring-bt-navigator.html)
- [Nav2: huella completa](https://docs.nav2.org/configuration/packages/trajectory_critics/obstacle_footprint.html)
- [Nav2: mapa y localización](https://docs.nav2.org/setup_guides/sensors/mapping_localization.html)
- [Nav2: AMCL](https://docs.nav2.org/configuration/packages/configuring-amcl.html)
- [NVIDIA: LiDAR PhysX en Isaac Sim 5.1](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/sensors/isaacsim_sensors_physx_lidar.html)
- [Unitree: Point-LIO para sus LiDAR](https://github.com/unitreerobotics/point_lio_unilidar)
- [ROS 2 Jazzy: depth_image_proc](https://docs.ros.org/en/jazzy/p/depth_image_proc/doc/index.html)
- [Nav2: selección de ObstacleLayer y VoxelLayer](https://docs.nav2.org/tuning/index.html#costmap2d-plugins)
