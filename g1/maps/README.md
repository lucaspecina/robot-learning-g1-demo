# Mapas de navegación

`demo_room.yaml` y `demo_room.pgm` son el mapa fijo de la habitación vigente.
Se generaron con SLAM Toolbox, con el robot quieto en el origen, y se guardaron
con la herramienta oficial `nav2_map_server/map_saver_cli`.

El lanzamiento normal carga este mapa y usa AMCL para ubicar al robot. El modo
`bash run_demo.sh map on` es distinto: detiene navegación y localización antes
de volver a construir un mapa. Nunca deben existir dos procesos publicando la
corrección `map -> odom`.

El mapa no incluye el cajón de la puerta de rodeo. Se construyó con
`G1_NAVIGATION_OBSTACLE=0 bash run_demo.sh up`; después se activó SLAM con
`bash run_demo.sh map on` y se guardó con `nav2_map_server/map_saver_cli`.
La prueba exige que esa celda esté libre aquí y ocupada en el mapa vivo de
Nav2 antes de mover el robot. Así demuestra detección por sensor y no memoria.

El cajón actual es alto y cruza el plano 2D. Esto no demuestra detección de
obstáculos bajos: esa aceptación requiere la nube 3D y la `VoxelLayer` de
Nav2. La posición inicial `(0, 0, 0)` es conocida sólo porque la prueba siempre
nace en el mismo lugar; el robot real deberá iniciar desde una base conocida o
recibir una relocalización explícita.
