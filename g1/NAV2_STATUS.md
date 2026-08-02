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

El primer rodeo físico también aisló las responsabilidades correctamente:
Nav2 planeó alrededor de un cajón de 60 cm, el cuerpo dejó 58 cm de espacio,
la altura no bajó de 0,735 m y la cancelación devolvió el mando a `STAND`.
No se aprueba la navegación completa porque la localización informó que
faltaban 5 cm cuando el error físico todavía era 49 cm.

## Experimentos de localización móvil

| Cambio único | Resultado |
|---|---|
| SLAM online durante el rodeo | esquivó; corrección falsa de 0,93 m / 9,0° |
| mapa fijo + AMCL diferencial | corrección de 0,68 m / 11,8°; no llegó físicamente |
| AMCL omnidireccional, contrato correcto del G1 | mejoró a 0,42 m / 8,0°; error físico 0,49 m |
| mismo caso limitado a 0,15 m/s | localización 0,04 m / 0,7°, pero la policy casi no avanzó |

La velocidad lenta no sirve como arreglo: cayó en la zona de órdenes que
mueven las piernas sin desplazar útilmente el cuerpo y fue revertida. El
modelo omnidireccional sí se conserva porque el contrato de la policy acepta
movimiento lateral y la deriva lateral está medida.

## Lo que todavía no está aprobado

- terminar físicamente a 15 cm después de rodear el obstáculo;
- repetición e inspección visual del rodeo completo;
- localización móvil transferible: Isaac todavía entrega odometría perfecta y
  la T4 no compensa bien el movimiento del barrido RTX;
- obstáculos bajos: la puerta actual cruza el plano 2D a la altura del LiDAR;
  falta alimentar la capa 3D oficial de Nav2 con `PointCloud2`;
- huella dinámica cuando brazos y carga sobresalen del torso.

No se seguirá afinando tolerancias de Nav2 para ocultar ese error. El próximo
experimento útil es repetir sin deformación móvil: GPU Ampere o posterior, o
el LiDAR PhysX oficial de Isaac 5.1 manteniendo el mismo contrato ROS. Para el
robot real, el camino de referencia es LiDAR + IMU con Point-LIO de Unitree.

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
