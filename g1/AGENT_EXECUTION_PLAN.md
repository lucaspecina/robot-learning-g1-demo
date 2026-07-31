# Plan de ejecución adaptable del agente

Estado de esta decisión: **ejecución adaptable con evidencia visual puntual y
búsqueda activa implementada y verificada**, 31-jul-2026.

Este documento existe para poder retomar el trabajo en una sesión nueva sin
reconstruir decisiones desde el chat. Describe el estado actual, el diseño que
vamos a implementar y lo que permitirá demostrar.

## Actualización posterior: balanceo de transporte cuantificado

Último código probado: `b192999`. La VM quedó encendida, Isaac activo, el
robot congelado en el origen y la carga retirada.

Lucas aprobó la forma congelada de la nueva pose, pero rechazó visualmente la
caminata con `0,5 kg` por balanceo excesivo. El recorrido y el frenado pasaban
los límites viejos, que no medían la calidad del movimiento del torso. Por eso
`checks.py walk` ahora registra inclinación lateral y frontal, velocidad
angular y rebote vertical durante caminata y frenado. Pasaron `125` pruebas de
`g1` y `21` del servidor, `146` en total.

Se hicieron tres parejas controladas con la misma pose, velocidad y duración;
la única variable fue `0` frente a `0,5 kg`. La mediana del balance lateral al
caminar fue `8,7°` sin peso y `8,3°` con peso; los peores casos fueron `9,3°` y
`9,7°`. La inclinación total p95 tuvo la misma mediana (`5,8°`) y el peso elevó
el balance frontal mediano de `2,6°` a `3,1°`. La desviación lateral mediana sí
empeoró de `4 cm` a `8 cm`, y el rumbo final mediano de `1°` a `6°`.

Conclusión: el peso degrada algo la precisión, pero no explica el balanceo
grande. La marcha base con esta pose ya oscila aproximadamente `8–10°` de lado
a lado. La locomoción no se cayó, pero la prueba integral de transporte queda
**rechazada visualmente** y no debe lanzarse otra misión completa. El próximo
experimento debe separar pose de brazos y carga: medir `reposo`, `transporte`
sin carga y `transporte` con `0,5 kg`, sin cambiar otra variable.

## Cierre de sesión del 31-jul-2026

Último código probado: `498a15d`. La VM se apagó después de este cierre.

La misión integral llegó a ejecutar alineación, carga de `0,5 kg`, pose de
transporte y regreso. La inspección de Lucas encontró dos defectos que los
verificadores numéricos no detectaban:

1. el tablero mezclaba la historia de misiones anteriores;
2. la pose que nosotros llamábamos `transporte` llevaba el bulto contra la
   pelvis y no parecía un transporte real, aunque las articulaciones hubieran
   alcanzado exactamente esos ángulos.

El tablero quedó corregido y comprobado en vivo: cambia de historia al cambiar
`mission_id`, vuelve a cero en `idle` y muestra `/g1/arm_status`, la medición
real, en vez de repetir la orden. Pasaron `122` pruebas de `g1` y `21` del
servidor, `143` en total. Antes de apagar, el tablero mostraba cero eventos,
`reposo · confirmada` y carga retirada.

NVIDIA no ofrece una pose fija llamada transporte en este flujo: su entorno
oficial de locomanipulación controla la posición de ambas muñecas mediante
cinemática inversa. Como escalón temporal sin manos, se creó una candidata
bilateral espejando el brazo que transporta un objeto en el cuadro 178 de la
demostración oficial
`object_pick_and_place_retarget_motion_g1_3finger_hands.yaml`.

La vista previa congelada de esa candidata midió:

- error articular máximo: `1,1°`;
- separación entre muñecas: `42,2 cm`;
- centro del bulto: `26,9 cm` delante y `8,3 cm` por encima de la pelvis;
- distancia total entre pelvis y centro del bulto: `28,1 cm`;
- masa: `0,50 kg`, `0,25 kg` verificados en cada muñeca.

**La candidata todavía no está aprobada visualmente ni físicamente.** Lucas
pidió apagar antes de juzgarla. Al retomar:

1. encender VM, abrir Isaac y el tablero;
2. con el robot congelado, ejecutar `pose transporte` y `payload attach 0.5`;
3. Lucas debe confirmar que el bulto está realmente entre las manos, delante
   del cuerpo y sin atravesarlo;
4. si aprueba visualmente, retirar la vista previa y ejecutar quietud, caminata
   y frenado con `0,5 kg`;
5. recién después repetir navegación y la misión integral.

No lanzar directamente otra misión completa: cambiar los ángulos de brazos es
un cambio físico y debe volver a pasar esa escalera.

## Decisión resumida

El modelo crea un plan inicial, pero el robot no lo obedece completo a ciegas.
La Jetson ejecuta una sola capacidad, recoge su resultado medido y vuelve a
consultar al modelo. El modelo puede conservar o modificar únicamente la parte
pendiente del plan.

Por ahora se usarán **imágenes puntuales tomadas por evento**, no video remoto.
La cámara y los detectores continúan trabajando localmente; el servidor recibe
una imagen completa o un recorte sólo cuando aporta evidencia para una
decisión.

Se postergan deliberadamente:

- Gemini Robotics VLA o cualquier otra VLA de cuerpo completo;
- video continuo hacia el modelo;
- Nav2 y SLAM;
- agarre aprendido.

Nav2 es el sistema estándar de navegación de ROS 2. SLAM es construir un mapa
con sensores y ubicar al robot dentro de él. Ambos reemplazarán después la
navegación simple sin cambiar el contrato del agente.

## Dos bloques siguientes, después de la validación visual actual

No se agregarán más capacidades a la misión inmediatamente después de cerrar
la corrida visual. Primero se harán estos dos bloques, en este orden:

1. **Auditoría de preparación para el robot real.** Inventariar todo mecanismo
   de respaldo, dato perfecto de Isaac, valor fijo, detector específico de la
   escena y simulación de una capacidad que todavía no existe. Cada elemento se
   clasificará como:
   - simulación válida de una interfaz que tendrá el robot real;
   - inyección de prueba, permitida sólo en un perfil de prueba explícito;
   - atajo incompatible con despliegue, que debe desaparecer del camino normal.
2. **Mapa, ubicación y navegación estándar.** Retomar LiDAR, SLAM y Nav2 desde
   el dato crudo del sensor hasta una misión completa, sin leer la posición
   perfecta que conoce Isaac. El resultado debe poder reemplazar la navegación
   simple sin cambiar las capacidades que consume el agente.

El primer candidato que debe salir del camino normal es el plan local de
respaldo: si Azure no responde o devuelve algo inválido, el robot real debe
quedar seguro, pedir ayuda o detener la misión. Ejecutar silenciosamente una
misión conocida haría parecer inteligente y conectado a un sistema que en
realidad falló. El respaldo podrá conservarse como prueba explícita y visible,
pero no activarse por defecto.

Tampoco se llamará “fake” a todo lo simulado. Por ejemplo, un LiDAR de Isaac
que publica el mismo tipo de medición que el sensor físico es una simulación
útil. En cambio, usar coordenadas internas perfectas de la escena para ubicar
el robot o un objeto sólo valida el guion, no el sistema que se desplegará.

## Estado actual

Ya funciona:

- catálogo cerrado con descripción, argumentos, condiciones y efectos de cada
  capacidad;
- plan inicial generado por `gpt-4.1-mini`;
- formato JSON estricto;
- validación independiente en el servidor y en la Jetson;
- plan local de respaldo para la misión conocida;
- trazabilidad de entrada exacta, salida literal y plan aceptado;
- ejecución secuencial del plan;
- revisión remota después de cada paso y después de cada falla;
- decisiones `continue`, `complete`, `retry`, `revise`, `ask_human` y `stop`
  validadas en el servidor y nuevamente en la Jetson;
- un único reintento por paso, impuesto localmente aunque el modelo pida más;
- reemplazo exclusivo de pasos pendientes sin borrar el historial;
- imágenes puntuales enlazadas por la fecha exacta del sensor para
  `look_at`, `read_clock`, `search_table` y `scan_for_table`;
- barrido activo de cinco vistas superpuestas con giro cancelable, detector
  local como filtro y Grounding DINO como confirmación;
- tablero con plan inicial, última revisión e intercambio literal del modelo.

La prueba real original produjo los 11 pasos correctos en 4,3 segundos. La
prueba adaptable mínima guardó `home`, recibió `continue` del modelo en
1,1–1,3 segundos y regresó a la misma pose. En una falla inducida, el modelo
pidió `retry`, el agente lo permitió una sola vez y detuvo la misión cuando
volvió a fallar. El robot permaneció en `STAND` en ambos casos.
Las 94 pruebas locales de `g1` y las 19 del servicio externo pasan juntas.
El tablero tampoco solicita imágenes inexistentes: espera la confirmación del
servidor y mantiene un estado vacío estable.

El 31-jul se volvió a probar el ciclo contra Azure, sin respuestas simuladas.
`gpt-4.1-mini` armó un plan de un paso para “recordá este lugar”, la Jetson
guardó `home`, el modelo revisó la medición y cerró la misión con `complete`.
El tablero recibió dos eventos reales —planificación y revisión— con entrada,
salida literal y resultado validado; no expuso nombres de variables secretas
ni claves. La aprobación visual de Lucas continúa pendiente.

La misión completa hasta localizar la mesa pasó dos veces. En la primera
corrida llegó al reloj con 11,7 cm de error, confirmó el reloj con 0,949, leyó
`09:00` y encontró la mesa roja en la cuarta vista. Hubo un solo candidato,
una sola llamada remota y la superficie medida cayó en `(3,674; 2,370; 0,671)`
m; el barrido desplazó la base 17,1 cm y terminó en `STAND`. La segunda corrida
volvió a encontrarla en la cuarta vista, en `(3,65; 2,78)` m, y enlazó el JPEG
exacto de 25.320 bytes a la revisión final.

Esa segunda corrida descubrió una ambigüedad del contrato: el modelo escribió
que la misión estaba cumplida pero sólo podía elegir `continue` o `stop`, y
eligió `stop`. Se agregó `complete` como cierre exitoso explícito. Una llamada
real posterior con `gpt-4.1-mini` devolvió `complete`; servidor y Jetson
rechazan usarlo después de una falla o si todavía quedan pasos.

La preaproximación a la mesa también quedó cerrada como etapa independiente.
El primer diseño pidió 0,9 m desde un punto visible de la superficie: una
corrida quedó a 0,543 m y otras perdieron la mesa del cuadro. La prueba
dedicada de cámara ya había medido que la vista estable estaba cerca de 2,5 m
del centro. Con 2,2 m desde la superficie visible, una misión integral llegó
con 9,8 cm de error, volvió a detectar la mesa a 1,968 m con confianza 0,94,
conservó 0,737 m de altura y preparó los brazos con 0,0254 rad de error
máximo. En esa corrida el agente se bloqueó después porque
`align_with_table` y `grasp_object` estaban declaradas como no disponibles.
`align_with_table` ya está disponible mediante `DockRobot`: roja terminó a
1,6 cm y 1,58°, y azul a 0,2 cm y 0,25° después de un reintento. El plan usa
después `attach_payload`, una carga simulada de 0,5 kg que pasó tres caminatas
y tres navegaciones y no se presenta como agarre.

El 31-jul todos esos pasos corrieron juntos por primera vez. El plan real de
Azure guardó `home`, llegó al reloj, confirmó 0,952, leyó `09:00`, eligió la
mesa roja, agregó un barrido al no verla, la ubicó con confianza 0,862, volvió
a medirla en la preaproximación, encontró el objeto, alineó a 2,9 cm y 1,96°,
aplicó 0,5 kg, puso los brazos en transporte y regresó. La alineación usó
`requested_pose`, no una detección local fingida, y no necesitó reintentos.

Esa corrida reveló a la vez que el regreso aceptaba 18,8 cm porque nuestro
margen recordado duplicaba los 10 cm declarados. Se cambió sólo ese margen al
valor oficial por defecto de Nav2, cero. El A/B posterior, todavía con 0,5 kg,
regresó a 8,3 cm y 4,5°, con 0,764 m de altura. Falta que Lucas observe una
corrida completa con este último commit; los tramos numéricos ya están
validados.

Todavía no funciona:

- migrar al contrato cancelable las capacidades distintas de navegación;
- presentar y responder la pregunta del operador cuando la decisión sea
  `ask_human`;
- validar visualmente con Lucas el nuevo tablero.

## Avance verificado del 30-jul-2026

La navegación ya usa la Action estándar `nav2_msgs/NavigateToPose` bajo el
nombre `/g1/navigate_to_pose`. El cálculo del movimiento sigue siendo nuestro
navegador simple, pero el agente ya no depende de un texto suelto para saber si
terminó.

Quedó medido:

- éxito sin desplazamiento: dos mensajes de progreso y dueño final `STAND`;
- cancelación durante movimiento: 11 mensajes y dueño final `STAND`;
- robot sin progresar: abortó tras 3 segundos y 32 mensajes;
- plazo total vencido: abortó con 20 mensajes;
- proceso de navegación muerto: la concesión venció y volvió a `STAND`;
- prueba física: 7,3 cm de error final, altura 0,73 m y 788 mensajes;
- regresión física: quietud con 2 cm de error, caminata de 2,43 m y frenado en
  4 cm.

La imagen estable de ROS 2 Jazzy todavía no incluye los códigos numéricos
`TIMEOUT` y `UNKNOWN` que aparecen en versiones posteriores de Nav2. Por eso
se usa el estado estándar `ABORTED` junto con `error_msg`; no se inventaron
códigos locales incompatibles.

La evidencia visual puntual se verificó también contra el sistema real. La
revisión posterior a `look_at` recibió el cuadro completo exacto, reconoció el
reloj y leyó visualmente `09:00`. `read_clock` recibió sólo el recorte,
devolvió `09:00` y la revisión siguiente continuó el plan. El JPEG enviado se
republica en `/g1/model_input/compressed` sólo para observación; nunca entra al
control del cuerpo.

La misma corrida encontró un límite útil: al faltar el barrido visual, el
modelo intentó sustituirlo por navegación entre puntos conocidos. Era un plan
formalmente válido pero físicamente inútil. Ahora se distinguen dos casos. Si
una capacidad realmente falta (`missing_skill`), servidor y Jetson sólo
aceptan pedir ayuda o detener la misión. Si existe una recuperación declarada
(`recoverable_with_skill`), ambos exigen `revise` y que el plan nuevo comience
con esa capacidad. Una llamada real cambió `search_table` por
`scan_for_table`; otra pidió ayuda correctamente cuando `align_with_table`
seguía como `placeholder`.

## Arquitectura objetivo de este tramo

```text
orden de la persona
        ↓
plan inicial propuesto por el modelo
        ↓
validación local completa
        ↓
ejecutar UNA capacidad
        ↓
resultado + progreso + mediciones + imagen opcional
        ↓
primero dejar el robot en un estado local seguro
        ↓
modelo: continuar / completar / repetir / modificar / pedir ayuda / detener
        ↓
validar otra vez solamente la parte pendiente
        ↓
siguiente capacidad
```

El modelo nunca cancela motores directamente. La Jetson detecta el problema,
cancela la capacidad y devuelve la autoridad a `STAND` antes de consultar al
servidor. `STAND` es la espera activa que mantiene de pie al bípedo.

## Contrato obligatorio de una capacidad

Cada capacidad larga debe exponer:

- objetivo solicitado;
- confirmación de aceptación o rechazo;
- progreso medible durante la ejecución;
- resultado final estructurado;
- plazo máximo;
- cancelación;
- procedimiento local para terminar en estado seguro.

Este contrato coincide con las **Actions de ROS 2**: la interfaz estándar para
tareas largas que tienen objetivo, progreso, resultado y cancelación. Los
topics se conservan para sensores y datos continuos. La navegación será la
primera capacidad migrada; cuando llegue Nav2, su servidor reemplazará nuestra
implementación detrás del mismo tipo de contrato.

## Supervisión local

La Jetson, no el modelo remoto, decide si una capacidad sigue viva.

Ejemplos iniciales:

| Capacidad | Progreso local | Éxito | Ante bloqueo |
|---|---|---|---|
| `navigate_to` | baja la distancia restante o mejora el ángulo | pose dentro de tolerancia durante el tiempo exigido | cancelar, liberar movilidad y pasar a `STAND` |
| `look_at` | llegan imágenes recientes y detecciones del objetivo | objetivo confirmado con umbral válido | terminar sin movimiento y devolver evidencia |
| `read_clock` | pedido remoto aceptado y respuesta dentro del plazo | lectura estructurada y consistente | abrir el circuito remoto y conservar `STAND` |
| `search_table` | llegan cuadros y resultados de búsqueda | mesa elegida localizada con profundidad válida | bloquear hasta tener barrido activo |
| `approach_table` | baja la distancia a la pose gruesa y llega una detección nueva | superficie a 1,8–2,8 m, base orientada y cuerpo en pie | volver a `STAND`; como máximo dos intentos por misión |
| `set_arm_pose` | disminuye el error de articulaciones | pose medida dentro de tolerancia | detener el pedido de brazo sin iniciar locomoción |
| `find_object` | llega una detección 3D posterior al inicio del paso | superficie visible a menos de 0,75 m del punto de la mesa | fallar sin mover la base; nunca convertir el punto en una pose de agarre |
| `attach_payload` | llega una confirmación con el mismo `request_id` | PhysX relee la masa pedida en exactamente dos puntos | rechazar la misión; nunca afirmar que hubo agarre |

Los plazos se medirán con reloj simulado cuando esté disponible. Mientras el
simulador siga lento, cada prueba declarará explícitamente si usa tiempo de
simulación o un margen transitorio de tiempo de pared.

Habrá límites duros para evitar ciclos infinitos:

- máximo 20 decisiones por misión en la primera versión;
- un reintento automático del mismo paso ante el mismo error;
- máximo dos barridos completos por misión; al agotarlos se declara que falta
  `relocate_viewpoint`, una capacidad que cambie físicamente el punto de
  observación. El validador sólo admite pedir ayuda o detenerse: cambiar el
  texto de otro barrido no cuenta como movimiento;
- máximo dos preaproximaciones por misión, aunque el modelo cambie el nombre
  del paso al revisar el plan;
- una respuesta inválida del modelo no reemplaza el plan validado;
- un corte de red nunca impide cancelar localmente ni mantener `STAND`.

Estos valores son guardas iniciales, no métricas declaradas como óptimas.

## Qué recibe el modelo después de cada paso

Siempre:

- misión original;
- plan aceptado;
- pasos completados;
- paso recién ejecutado;
- resultado, error y mediciones;
- hechos del mundo actualmente comprobados;
- catálogo vigente de capacidades.

Sólo cuando corresponda:

- `look_at`: imagen completa con la detección marcada;
- `read_clock`: recorte exacto del reloj;
- `search_table`: imagen completa para contexto y recorte de la mesa;
- futuro agarre: imagen anterior y posterior.

No se enviará una imagen por costumbre. Navegar correctamente puede
comprobarse con posición y estado; una imagen sólo se agrega cuando responde
una pregunta visual.

## Respuesta permitida del modelo

La respuesta tendrá formato estricto y una de estas decisiones:

- `continue`: conservar el plan pendiente;
- `complete`: cerrar con éxito sólo si el último paso tuvo éxito y no queda
  ningún paso pendiente;
- `retry`: repetir la capacidad recién fallada, dentro del límite local;
- `revise`: reemplazar únicamente los pasos todavía no ejecutados;
- `ask_human`: pasar a `STAND` y mostrar una pregunta concreta;
- `stop`: terminar la misión indicando el motivo.

La Jetson rechaza cualquier revisión que:

- invente una capacidad;
- cambie pasos ya completados;
- use argumentos desconocidos;
- viole condiciones previas;
- supere límites de pasos o reintentos;
- intente ejecutar dos capacidades físicas simultáneamente.

## Orden de implementación

1. ~~Crear el contrato común de ejecución y los estados de cancelación.~~
2. ~~Migrar `navigate_to` y medir distancia, progreso, éxito y bloqueo.~~
3. ~~Probar tres fallas inducidas: falta de progreso, proceso muerto y plazo
   vencido; en las tres debe terminar en `STAND`.~~
4. ~~Agregar la revisión remota después de cada paso, inicialmente sin
   imagen.~~
5. ~~Validar de nuevo los pasos pendientes y conservar el plan anterior si la
   revisión es inválida.~~
6. ~~Adjuntar imágenes puntuales a `look_at`, `read_clock` y
   `search_table`.~~
7. ~~Agregar `scan_for_table` con giro cancelable, vistas nuevas y
   confirmación remota sólo ante candidatos.~~ Dos corridas físicas pasaron.
8. Migrar brazos y demás capacidades listas al mismo contrato.
9. ~~Mostrar en el tablero plan original, revisión, evidencia, cancelación y
   plan vigente.~~ Falta la aprobación visual de Lucas.
10. Repetir la misión con red limpia, red lenta y corte total.
11. Hacer la validación visual con Lucas.

Ningún cambio de esta lista modifica física ni locomoción. Por eso exige
pruebas funcionales del agente y una inspección visual, pero no repetir toda la
bisección física salvo que se toque control corporal.

## Demo disponible al cerrar este tramo

La persona enviará por texto la misión; la voz se agregará después. El modelo
armará el plan inicial y el tablero lo mostrará. El robot:

1. guardará `home`;
2. irá al reloj conocido;
3. comprobará con una imagen que realmente lo está mirando;
4. leerá la hora desde su recorte;
5. elegirá la mesa roja o azul mediante la regla determinista;
6. buscará la mesa alrededor con cinco vistas superpuestas y sólo llamará al
   detector remoto ante un candidato local;
7. calculará una preaproximación, navegará hasta ella y volverá a medir la
   mesa antes de preparar los brazos;
8. revisará después de cada paso, incluido el último, si debe continuar,
   cambiar el plan o declarar la misión completada;
9. cancelará cualquier paso trabado y quedará activamente en `STAND`;
10. mostrará la entrada, respuesta, mediciones e imagen usadas para decidir.

El sistema todavía se bloqueará honestamente si no encuentra la mesa después
del barrido o si necesita cambiar físicamente el punto de observación. La
alineación y el transporte con carga simulada ya funcionan; el agarre real
sigue faltando y nunca se presenta la masa agregada como prueba de dedos,
contacto o retención.

Por lo tanto, este tramo completa el recorrido de transporte simulado, no la
entrega física del objeto. Convierte la demo de una lista rígida en una misión
observable que detecta fallas, se pone
segura y puede corregir la parte ejecutable de su plan.

## Referencias oficiales usadas

- Google Gemini Robotics ER 2, capacidades y arquitectura:
  https://ai.google.dev/gemini-api/docs/robotics-overview
- Google, ciclo de ejecución y devolución de resultados:
  https://ai.google.dev/gemini-api/docs/robotics-orchestration
- Google, capacidades declaradas con nombre, descripción y parámetros:
  https://ai.google.dev/gemini-api/docs/robotics-streaming
- ROS 2 Jazzy, cuándo usar topics, servicios y Actions:
  https://docs.ros.org/en/jazzy/How-To-Guides/Topics-Services-Actions.html
- Gemini Robotics 2, seguridad: el modelo no reemplaza protecciones físicas:
  https://storage.googleapis.com/deepmind-media/gemini-robotics/Gemini-Robotics-2-Safety.pdf
- ROS 2, visualización de imágenes, diagnósticos y logs con Foxglove:
  https://docs.ros.org/en/rolling/Related-Projects/Visualizing-ROS-2-Data-With-Foxglove.html
- OpenTelemetry, contenido completo opcional de pedidos y respuestas de
  modelos generativos:
  https://opentelemetry.io/blog/2026/genai-observability/
- OpenAI, imágenes por URL Base64 y nivel de detalle para OCR:
  https://developers.openai.com/api/docs/guides/images-vision
- Nav2, navegación a una pose de espera, nueva detección y control visual
  refinado:
  https://docs.nav2.org/tutorials/docs/using_docking.html
- Nav2, plazos, reintentos y separación configurable de la pose de espera:
  https://docs.nav2.org/configuration/packages/configuring-docking-server.html
- Nav2, recuperación y replanificación mediante Behavior Trees:
  https://docs.nav2.org/behavior_trees/index.html
- Nav2, flujo recomendado de mapa, localización y mapas de obstáculos:
  https://docs.nav2.org/setup_guides/sensors/mapping_localization.html
- NVIDIA Isaac Sim, LiDAR RTX publicado mediante mensajes ROS 2 estándar:
  https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/tutorial_ros2_rtx_lidar.html
- Unitree, sensores declarados para el G1 EDU:
  https://www.unitree.com/g1/
- Google SayCan, selección de capacidades limitada por lo que el robot puede
  ejecutar realmente:
  https://research.google/blog/towards-helpful-robots-grounding-language-in-robotic-affordances/

## Qué es oficial y qué es adaptación propia

Coincide con el flujo oficial:

- capacidades declaradas con descripción y argumentos;
- ejecutar, devolver resultado y volver a decidir;
- capacidades físicas bloqueantes: no pedir otra hasta que terminen;
- progreso y cancelación locales para tareas largas;
- protecciones deterministas debajo del modelo.

Adaptación propia de esta demo:

- Azure `gpt-4.1-mini` en lugar de Gemini Robotics ER 2;
- plan inicial completo y revisiones por evento;
- imágenes puntuales en lugar de video continuo;
- topics actuales que se migrarán gradualmente a Actions;
- navegación simple hasta incorporar Nav2 y SLAM;
- preaproximación inspirada en el flujo de Nav2 Docking, pero con nuestra
  distancia medida y sin afirmar que el servidor oficial ya está integrado;
- AGILE conserva locomoción y balance; el modelo no controla el cuerpo.
