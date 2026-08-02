# Mapas de navegación

`demo_room.yaml` y `demo_room.pgm` son el mapa fijo de la habitación vigente.
Se generaron con SLAM Toolbox, con el robot quieto en el origen, y se guardaron
con la herramienta oficial `nav2_map_server/map_saver_cli`.

El lanzamiento normal carga este mapa y usa AMCL para ubicar al robot. El modo
`bash run_demo.sh map on` es distinto: detiene navegación y localización antes
de volver a construir un mapa. Nunca deben existir dos procesos publicando la
corrección `map -> odom`.

El mapa incluye el cajón alto de la puerta de rodeo. No demuestra detección de
obstáculos bajos: esa aceptación requiere la nube 3D y la `VoxelLayer` de
Nav2. La posición inicial `(0, 0, 0)` es conocida sólo porque la prueba siempre
nace en el mismo lugar; el robot real deberá iniciar desde una base conocida o
recibir una relocalización explícita.
