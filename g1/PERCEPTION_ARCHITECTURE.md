# Percepción: qué ve el robot y qué pasa con cada imagen

La imagen de **Lo que ve el robot** es la imagen RGB completa de su cámara
frontal. El tablero sólo agrega una franja negra con el número de cuadro y
comprime la imagen como JPEG para mostrarla en el navegador. No existe otra
vista más amplia que el robot use a escondidas.

## Flujo actual

```text
cámara frontal
      |
      +--> RT-DETR local continuo --> `/g1/object_detections` --------+
      |          + señal amplia de color                               |
      |                         |                                      |
      |                    candidato barato                            |
      |                         v                                      |
      +--> barrido de cinco vistas con `/g1/spin`                      |
      |                                                               |
      +--> pedido puntual --> servidor Grounding DINO                 |
                              --> `/g1/open_vocabulary_detections` ----+
                                          |
 color + profundidad + calibración + pose histórica de cámara
                                          |
                                          v
                                localizador de mesas
                                `/g1/table_detections_3d`
                                                                      v
                                                          adaptador de la demo
      |
      +--> reloj / botella / mesa roja o azul en `/g1/detections`
      |
      +--> sólo el recorte del reloj en `/g1/clock_crop/compressed`
                                |
                                v
                    modelo visual remoto, al leer la hora

      +--> cuadro enlazado en `/g1/perception/evidence/compressed`
                                |
                 sólo si una revisión visual lo necesita
                                v
                    `/g1/model_input/compressed`
                    + pedido remoto de revisión
```

RT-DETR es un modelo liviano que encuentra objetos conocidos y devuelve un
rectángulo para cada uno. Corre a bordo y no necesita Internet. El modelo
remoto recibe únicamente un recorte cuando la tarea requiere entender un
detalle, por ejemplo leer los dígitos del reloj.

Grounding DINO es otro modelo remoto. Sólo se activa por un pedido interno
acotado (`red_table` o `blue_table`) y recibe un cuadro JPEG. Encuentra la
clase general `mesa`; no decide el color. Una prueba le pidió “mesa azul” sobre
una mesa roja y respondió incorrectamente con confianza 0,805. Por eso el
adaptador mide rojo o azul dentro del recuadro. El detector local y el conteo
amplio de píxeles de color sólo deciden si vale la pena hacer esa llamada:
nunca pueden declarar que la mesa fue encontrada. Navegar y quedarse de pie
no dependen de ninguno de estos modelos ni de la red externa.

## Memoria de imágenes

No se graba video infinito:

- El detector conserva sólo el cuadro más nuevo mientras trabaja. Si llegan
  más, descarta los viejos para no tomar decisiones atrasadas.
- El adaptador conserva en RAM hasta 180 cuadros identificados por su hora,
  aproximadamente un minuto a 3 cuadros por segundo. Esto permite unir una
  respuesta lenta con la imagen exacta que la produjo.
- El localizador conserva hasta 120 juegos de color, profundidad y calibración,
  y 120 segundos de posiciones de cámara. No aproxima horarios: si falta el
  instante exacto, rechaza la medición.
- El video general tolera perder cuadros, pero el localizador 3D escucha
  color, profundidad y calibración con la misma entrega garantizada que usa
  la cámara. Antes de igualar ese contrato sólo pudo reconstruir 3 de 7
  llamadas exitosas; después conservó 3 de 3 repeticiones consecutivas.
- El agente conserva sólo el último recorte del reloj y lo considera vencido
  después de 10 segundos.
- El agente conserva hasta 24 cuadros de evidencia en RAM. Cuando una
  revisión realmente necesita uno, exige la misma fecha del sensor, lo envía
  una sola vez y republica ese JPEG exacto para el tablero.
- El tablero usa otro historial acotado de 180 cuadros para dibujar las cajas
  sobre el cuadro correcto. Conserva el último resultado puntual durante un
  minuto para que el operador alcance a inspeccionarlo. No escribe esas
  imágenes en disco.

Para investigar una falla concreta se podrá grabar una corrida acotada con las
herramientas de ROS 2. La grabación permanente no debe ser el modo normal del
robot físico por almacenamiento y privacidad.

## Qué puede observar Lucas

El tablero muestra:

1. el video vivo completo;
2. el último cuadro que realmente analizó el detector, con cajas y confianza;
3. la imagen exacta enviada al modelo remoto;
4. el nombre del modelo y el tiempo de la llamada;
5. el texto literal que devolvió, sin resumirlo ni corregirlo;
6. el dato estructurado que aceptó el validador antes de que el robot actuara;
7. qué paso de la misión está activo, qué decisión se tomó y con qué evidencia.

Grounding DINO publica un evento propio antes de salir por HTTP y lo actualiza
con éxito o falla al terminar. El evento enlaza el JPEG publicado en
`/g1/model_input/compressed`; ambos usan la fecha original de adquisición de
la cámara. Así una respuesta perdida no borra qué cuadro se intentó enviar.

Esto permite distinguir cuatro casos diferentes: el objeto no entró en la
cámara, entró pero el detector no lo reconoció, fue reconocido pero la tarea
no reaccionó, o falló el modelo remoto. También deja visible una quinta falla:
que el texto del modelo sea correcto pero nuestro validador lo interprete mal.

Las revisiones generales usan detalle bajo para limitar demora y costo. El
reloj usa detalle alto porque sus dígitos son pequeños; es la recomendación
oficial para lectura de texto y objetos pequeños. No se envía una imagen al
revisar navegación, guardado de `home` ni decisiones puramente numéricas.

## Coincidencia con los flujos oficiales

| Parte | Referencia | Estado local |
|---|---|---|
| Cámara G1 simulada | Unitree: 7,6 mm, apertura 20 mm, 640×480, recta | mismos valores principales; montaje sobre nuestro `head_link` y 3 Hz son adaptaciones que deben medirse |
| Detección | NVIDIA Isaac ROS ofrece RT-DETR y publica cajas estándar de ROS 2 | mismo modelo conceptual y mismo tipo de mensaje |
| Búsqueda de categorías nuevas | NVIDIA ofrece Grounding DINO con una descripción escrita y recomienda usarlo de forma intercalada por su costo | integrado a pedido en el servidor; no corre dentro del control |
| Ejecución en la VM | Isaac ROS actual requiere una GPU más nueva que la T4 | backend compatible en CPU dentro de la Jetson simulada |
| Uso de modelo grande | procesar sólo cuando una tarea lo necesita | recorte para el reloj; un cuadro para buscar una mesa |
| Ubicación 3D | `vision_msgs/Detection3DArray` y relaciones temporales `tf2` | la Jetson publica el punto observado en `map` con la hora del cuadro |

Antes de instalar el paquete acelerado en el G1 físico habrá que fijar una
combinación compatible entre la Jetson real, su versión de JetPack, ROS 2 e
Isaac ROS. “Es oficial” no garantiza que cualquier versión funcione con
cualquier Jetson.

## Experimentos de cámara del 29 de julio de 2026

| Cambio único | Resultado |
|---|---|
| cámara documentada como 20° abajo | el cálculo y la imagen probaron que miraba 20° arriba |
| cámara recta, lente anterior de 60° | mostró suelo y horizonte, pero dejó el reloj arriba del cuadro |
| cámara recta, lente oficial de unos 106° | reloj completo y centrado; 2/3 detecciones, todavía inestable |
| mesa con base maciza | entró completa en cuadro, pero visualmente parecía un cajón; 0/3 |
| base reemplazada por cuatro patas | ya parece mesa, pero a 320×240 sus partes son demasiado pequeñas; 0/3 |
| resolución oficial 640×480 | reloj 3/3, 0/3 falsos, RTF 0,23–0,24; la mesa entró completa pero RT-DETR quedó debajo del umbral |
| confianza cruda de RT-DETR sobre la mesa | `diningtable` fue la mejor clase, 0,574, con la caja correcta; también reveló una diferencia de nombre corregida |
| Grounding DINO pequeño, consulta “mesa roja / mesa azul” | mesa roja correcta a 0,618; 18,9 s en los dos CPU simulados, demasiado lento para ejecutarlo continuamente a bordo |
| servidor separado, consulta genérica “mesa” | mesa roja 0,897; evita confiar en el atributo de color del modelo |
| navegación + mesa roja + servidor | llegada a 0,115 m y 4,9°; roja 3/3, azul 0/3; 17,05 s |
| navegación + mesa azul + servidor | llegada a 0,104 m y 5,0°; azul 3/3, roja 0/3; 16,78 s |
| misma mesa azul, wifi malo | azul 3/3, roja 0/3; 15,62 s; cuerpo a 0,734 m y orden cero |
| enlace cortado | falla explícita en 14,30 s; `stand` conserva el control y el cuerpo queda a 0,734 m |
| brazos en `reposo` | las manos tapan dos esquinas grandes de la cámara |
| brazos en `listo` | manos fuera del cuadro; pose y cuerpo aprobados; mesa azul 3/3 y caja visualmente correcta |
| umbral RT-DETR general 0,25 | mesa visible 6/6 con confianza 0,285–0,408; pared vacía 0/5 incluso probando 0,15 |
| barrido activo de 360° | cinco vistas, 72° entre ellas, 36,1° de superposición; roja confirmada en la cuarta vista con una sola llamada remota |

Una prueba numérica no cierra la cámara hasta que Lucas confirme también que
la imagen y las cajas tienen sentido.

## De recuadro a coordenada: validado el 30 de julio de 2026

Una caja 2D dice dónde aparece la mesa en la foto, no dónde está en la
habitación. El G1 real incluye cámara de profundidad y LiDAR 3D según la ficha
de Unitree. El flujo transferible implementado es:

1. publicar color, profundidad y calibración de la misma cámara;
2. medir la distancia sólo en los píxeles rojos o azules del recuadro;
3. transformar ese punto al mapa usando la pose histórica del sensor;
4. publicar una `Detection3DArray` estándar en `/g1/table_detections_3d`.

Isaac Lab ofrece `distance_to_image_plane`, calibración y pose de cámara. Su
opción `update_latest_camera_pose` está desactivada por defecto por rendimiento;
encenderla mantuvo RTF 0,23–0,24. No se usa la coordenada interna del objeto.

| Prueba | Punto medido | Referencia física |
|---|---|---|
| mesa roja | `(3,63; 2,45; 0,52)` m | cayó dentro de su superficie |
| mesa azul | `(3,62; -2,53; 0,52)` m | cayó dentro de su superficie |
| nodo permanente, azul | `(3,63; -2,65; 0,52)` m, confianza 0,923 | coincidió con el verificador independiente |

El punto representa la superficie vista, no el centro completo de la mesa.

## Búsqueda activa validada el 30 de julio de 2026

La búsqueda usa la lente realmente configurada, de 108,1° horizontales. El
patrón mínimo con al menos 30° de superposición son cinco vistas separadas
72°; la superposición efectiva queda en 36,1°. Cada giro usa la Action
estándar `nav2_msgs/Spin`, por lo que tiene progreso, plazo y cancelación.

La corrida completa guardó el inicio, llegó al reloj, lo confirmó, leyó
`09:00`, eligió la mesa roja y la encontró en la cuarta vista. Grounding DINO
dio 0,819 y profundidad ubicó la superficie en `(3,674; 2,370; 0,671)` m.
Hubo un candidato local y una sola consulta remota. El cuerpo terminó en
`STAND`; el barrido desplazó su base 0,171 m.

La interfaz coincide con Nav2, pero nuestra implementación todavía no tiene
el mapa local de colisiones que usa el comportamiento `Spin` completo. Por
eso el barrido actual sólo es seguro en la sala abierta de la demo. Cuando
LiDAR y Nav2 estén validados, el servidor oficial podrá reemplazar este
adaptador sin cambiar el agente.

El umbral 0,70 mostrado en un flujo de manipulación de NVIDIA no se copió:
corresponde a otro modelo especializado. Para nuestro checkpoint general,
seis cuadros reales de mesa dieron 0,285–0,408 y cinco cuadros de pared no
produjeron mesas ni siquiera a 0,15. Se fijó 0,25 porque conservó 6/6 positivos
y 0/5 falsos en ese control. Es una calibración local medida, no un valor
“oficial”.

## De superficie a preaproximación: validado el 30 de julio de 2026

El flujo estándar de Nav2 Docking no navega directamente al contacto. Primero
calcula una pose de espera que todavía deja el objetivo visible, navega hasta
ella, vuelve a detectar y recién después entra en un lazo de control visual
que actualiza el objetivo continuamente.

Nuestra adaptación implementa sólo esa primera etapa. La profundidad entrega
un punto de la parte visible de la mesa, no su centro ni su orientación
completa. La base se coloca sobre la línea robot–punto a 2,2 m, queda mirando
hacia él y exige una detección nueva antes de declarar éxito. Un máximo duro de
dos intentos no puede ser eludido por una revisión del LLM.

La distancia no se copió de Nav2: se midió con esta cámara. A 0,9 m, una prueba
terminó a 0,543 m y otras perdieron la mesa del cuadro. En la validación
integral, la nueva etapa:

- llegó a la pose gruesa con 0,098 m de error;
- volvió a ubicar la superficie a 1,968 m y confianza 0,94;
- conservó 0,737 m de altura y terminó en `STAND`;
- preparó los brazos con 0,0254 rad de error máximo;
- se bloqueó honestamente antes de `align_with_table`, todavía pendiente.

Esto coincide con la separación oficial entre llegada gruesa y control visual
refinado. No es el servidor Docking de Nav2 ni una alineación de agarre: el
navegador actual sólo funciona en la habitación despejada y todavía no tiene
mapa local de obstáculos.

Referencias oficiales:

- Unitree G1, sensores: https://www.unitree.com/mobile/g1/
- Cámara y profundidad de Isaac Lab:
  https://isaac-sim.github.io/IsaacLab/develop/source/overview/core-concepts/sensors/camera.html
- Conversión oficial a puntos 3D:
  https://isaac-sim.github.io/IsaacLab/develop/source/how-to/save_camera_output.html
- Transformaciones temporales de ROS 2:
  https://docs.ros.org/en/jazzy/p/tf2/generated/doxygen/html/index.html
- Sincronización por fecha y QoS de ROS 2:
  https://docs.ros.org/en/jazzy/p/message_filters/doc/Tutorials/Approximate-Synchronizer-Cpp.html
- Mensaje 3D estándar de ROS 2:
  https://docs.ros.org/en/jazzy/p/vision_msgs/msg/Detection3D.html
- Detección de objetos de Isaac ROS:
  https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_object_detection/index.html
- Action de giro de Nav2:
  https://api.nav2.org/actions/rolling/spin.html
- Configuración del servidor de comportamientos de Nav2:
  https://docs.nav2.org/configuration/packages/configuring-behavior-server.html
- Flujo de preaproximación y refinamiento visual de Nav2 Docking:
  https://docs.nav2.org/tutorials/docs/using_docking.html
- Configuración y tolerancias de Nav2 Docking:
  https://docs.nav2.org/configuration/packages/configuring-docking-server.html
