# Plan para quitar ayudas y preparar navegación real

Estado: **en ejecución; sensores y barrera local validados, conductor Nav2 pendiente**.
Fecha: 2-ago-2026.

Este documento separa dos trabajos que no deben mezclarse: primero auditar los
atajos de la demo y después incorporar percepción del espacio, mapa,
localización y Nav2. La referencia es siempre el G1 físico; Isaac debe reemplazar
sensores y física, no regalarle respuestas al agente.

## Estado medido de esta migración

| Parte estándar | Estado actual | Límite honesto |
|---|---|---|
| LiDAR 3D en ROS 2 | pasa: tres nubes de 285 mil puntos y 359,9° | el perfil y montaje de Isaac son adaptadores, no réplica del sensor comprado |
| Vista plana `/scan` | pasa: 1.066 rayos, 359,4°, tres vueltas completas | Isaac 5.1 exige un segundo perfil simulado; en el G1 real se proyectará la nube física |
| Coordenadas del sensor | pasa quieto y caminando | Isaac entrega todavía la pose perfecta del torso; el hardware usará URDF y estimación local |
| SLAM Toolbox quieto | pasa: 138 × 148 celdas a 5 cm | la localización móvil sobre la T4 no alcanza todavía la precisión exigida |
| Mapa local Nav2 | pasa: 60 × 60 celdas, obstáculos y margen de seguridad | representa obstáculos, pero el navegador propio todavía no lo consulta |
| Collision Monitor de Nav2 | pasa: tres paradas a 0,50–0,51 m | huella fija; debe cambiar al mover brazos o transportar una carga |
| Planificador y controlador Nav2 | pendiente | `go_to.py` sigue yendo directo al objetivo; aún no rodea obstáculos |

Nada de esta tabla afirma navegación autónoma completa. Hoy el robot puede ver
una pared y detenerse antes de golpearla. Todavía no calcula un desvío para
rodearla.

## Qué significa "fake"

No todo lo simulado es una ayuda indebida. Un sensor simulado es válido cuando
publica la misma clase de medición que entregará el sensor físico y el resto del
sistema no sabe si viene de Isaac o del robot. En cambio, leer la posición
perfecta que conoce el simulador o completar una misión con un plan oculto sí
falsea el resultado.

Cada mecanismo quedará clasificado con una de estas cuatro etiquetas:

- **transferible**: se conserva igual en simulación y robot;
- **adaptador de simulación**: reemplaza un driver físico, detrás de una
  interfaz común;
- **instrumento de prueba**: sólo mide una hipótesis y nunca se habilita en el
  perfil de ensayo de despliegue;
- **ayuda indebida**: se elimina o se cambia por un fallo explícito.

## Auditoría que haremos

| Mecanismo actual | Lectura preliminar | Resultado exigido |
|---|---|---|
| Plan local de respaldo para la misión conocida | ayuda indebida si se activa en silencio | si Azure o el planificador fallan, detener la misión, conservar `STAND` y mostrar el fallo; nunca ejecutar un libreto oculto |
| Coordenada conocida del reloj | escalón temporal | obtenerla de una anotación hecha al preparar el mapa o encontrar el reloj con sensores; el agente no puede leer la escena de Isaac |
| `/g1/odom` y transformaciones calculadas desde la pose perfecta de Isaac | adaptador hoy, bloqueante para despliegue | producir `odom -> base_link` con estimación local y `map -> odom` con localización basada en sensores |
| Profundidad de Isaac | adaptador válido sólo si imita la cámara real | misma imagen de profundidad, calibración, marco y sincronización que el driver físico; prohibido consultar distancia interna de la escena |
| `attach_payload` | instrumento de prueba explícito | mantenerlo únicamente para ensayos de carga; el perfil de despliegue debe detenerse en “agarre no disponible” hasta tener contacto y retención reales |
| Publicación manual de `/g1/mission` | escalón temporal | reemplazarla por voz sin cambiar el contrato de texto que recibe el agente |
| Navegador propio `go_to.py` | implementación provisional | conservar su Action como contrato y reemplazar el movimiento interno por Nav2 |
| Posiciones, nombres o colores de la escena usados por el tablero | instrumentación permitida | demostrar con pruebas que no entran al agente, detectores, mapa ni navegador |
| Contenedores y degradación de red | ensayo transferible y necesario | conservarlos; seguridad, equilibrio y frenado deben seguir locales |

La auditoría no consistirá sólo en buscar palabras como `fallback` o
`mock`. Seguiremos cada dato desde su origen hasta la decisión física y
probaremos qué ocurre al quitarlo.

## Perfil de ensayo de despliegue

Crearemos un modo explícito `deployment_rehearsal`. En ese modo:

1. no hay plan fijo de respaldo;
2. no se leen coordenadas internas de Isaac;
3. no se puede adjuntar carga mágicamente;
4. cada sensor entra por el mismo contrato ROS 2 previsto para el hardware;
5. una capacidad ausente bloquea honestamente la misión;
6. un corte del servidor termina en espera segura, no en éxito aparente.

La misión completa con carga anexada seguirá existiendo como **ensayo de
locomoción con peso**, pero no contará como ensayo de despliegue ni como agarre.

## Camino estándar de LiDAR a Nav2

LiDAR es el sensor que mide la geometría alrededor del robot. Nav2 es el
sistema estándar de ROS 2 que usa un mapa y una estimación de la posición del
robot para calcular un camino y evitar obstáculos. SLAM es construir el mapa a
la vez que el robot estima dónde está.

El orden será:

1. **Confirmar el hardware**: antes de fijar parámetros, confirmar el modelo,
   montaje, frecuencia y topics del LiDAR y de la cámara que traerá el G1 EDU.
2. **Recuperar la medición cruda en Isaac**: el LiDAR simulado debe producir
   puntos no vacíos de forma repetible y publicarlos como
   `sensor_msgs/PointCloud2`. No se conecta una nube vacía a nada posterior.
3. **Igualar contratos**: simulador y driver físico deben publicar mediciones,
   tiempo y marcos equivalentes. Para navegación plana se evaluará una vista
   2D `sensor_msgs/LaserScan`; la nube 3D se conserva para obstáculos con
   altura.
4. **Estimar movimiento local**: en el G1 real, comenzar con el driver oficial
   del LiDAR L2 y su IMU, y reproducir primero Point-LIO, la referencia abierta
   de Unitree para odometría y mapa 3D. Fusionar piernas o visión sólo si una
   medición demuestra que mejoran el resultado. La salida continua publica
   `odom -> base_link`; puede derivar lentamente, pero no debe saltar.
5. **Construir y guardar el mapa**: para el banco 2D actual, recorrer el lugar
   con SLAM Toolbox, revisar el mapa y guardarlo. Para el L2 físico, proyectar
   a navegación el mapa obtenido de la cadena 3D medida.
6. **Ubicarse al encender**: cargar el mapa y comparar el localizador 2D AMCL
   de Nav2 contra la localización 3D elegida; adoptar el que cumpla las métricas
   sin conocer la pose perfecta del simulador.
7. **Navegar con Nav2**: el mapa global decide por dónde ir; el mapa local se
   actualiza con obstáculos actuales; el controlador produce velocidades para
   la policy de locomoción.
8. **Cerrar la seguridad**: todas las fuentes pasan primero por nuestra
   autoridad de movilidad. El monitor de colisiones de Nav2 queda como último
   filtro antes de `/cmd_vel`, siguiendo la recomendación oficial.
9. **Agregar significado**: LiDAR dice “hay una superficie”; la cámara dice
   “eso es un reloj” o “esa mesa es roja”. Las detecciones se transforman al
   mapa sin usar etiquetas internas del simulador.

La cadena objetivo queda así:

```text
LiDAR + IMU + odometría de piernas/cámara
                 |
          mapa y localización
                 |
orden semántica -> pose objetivo -> Nav2 -> autoridad -> monitor de colisión
                                                        |
                                                     /cmd_vel
                                                        |
                                             policy de locomoción
```

## Criterios para avanzar

- nube cruda válida en tres encendidos y durante quietud/caminata;
- árbol de marcos `map -> odom -> base_link -> sensores` sin saltos ni datos
  del simulador fuera de los adaptadores;
- mapa guardado que coincida visualmente con paredes y obstáculos medidos;
- localización repetida desde varias posiciones iniciales seguras;
- diez navegaciones con error, colisiones y recuperaciones declarados antes;
- corte de LiDAR, localización, Jetson y servidor probado por separado;
- inspección visual de Lucas además de los números.

## Fuentes oficiales de referencia

- [Nav2: conceptos y marcos requeridos](https://docs.nav2.org/concepts/index.html)
- [Nav2: mapa y localización](https://docs.nav2.org/setup_guides/sensors/mapping_localization.html)
- [Nav2: navegar mientras se construye el mapa](https://docs.nav2.org/tutorials/docs/navigation2_with_slam.html)
- [Nav2: combinar odometría e IMU](https://docs.nav2.org/setup_guides/odom/setup_robot_localization.html)
- [Nav2: monitor de colisiones como último filtro](https://docs.nav2.org/tutorials/docs/using_collision_monitor.html)
- [NVIDIA Isaac Sim: LiDAR RTX y publicación ROS 2](https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/tutorial_ros2_rtx_lidar.html)
- [Unitree: SDK ROS 2 del LiDAR](https://github.com/unitreerobotics/unilidar_sdk2)
- [Unitree: Point-LIO con LiDAR L2 e IMU](https://github.com/unitreerobotics/point_lio_unilidar)
- [Nav2: mapa local de obstáculos e inflación](https://docs.nav2.org/setup_guides/sensors/mapping_localization.html)
- [Nav2: configuración de Collision Monitor](https://docs.nav2.org/configuration/packages/collision_monitor/configuring-collision-monitor-node.html)
