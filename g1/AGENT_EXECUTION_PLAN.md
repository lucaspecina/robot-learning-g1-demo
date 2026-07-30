# Plan de ejecución adaptable del agente

Estado de esta decisión: **ejecución adaptable sin imágenes implementada y
verificada**, 30-jul-2026.

Este documento existe para poder retomar el trabajo en una sesión nueva sin
reconstruir decisiones desde el chat. Describe el estado actual, el diseño que
vamos a implementar y lo que permitirá demostrar.

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
- decisiones `continue`, `retry`, `revise`, `ask_human` y `stop` validadas en
  el servidor y nuevamente en la Jetson;
- un único reintento por paso, impuesto localmente aunque el modelo pida más;
- reemplazo exclusivo de pasos pendientes sin borrar el historial;
- tablero con plan inicial, última revisión e intercambio literal del modelo.

La prueba real original produjo los 11 pasos correctos en 4,3 segundos. La
prueba adaptable mínima guardó `home`, recibió `continue` del modelo en
1,1–1,3 segundos y regresó a la misma pose. En una falla inducida, el modelo
pidió `retry`, el agente lo permitió una sola vez y detuvo la misión cuando
volvió a fallar. El robot permaneció en `STAND` en ambos casos.
Las 61 pruebas locales de `g1` y las 12 del servicio externo pasan juntas.
El tablero tampoco solicita imágenes inexistentes: espera la confirmación del
servidor y mantiene un estado vacío estable.

Todavía no funciona:

- migrar al contrato cancelable las capacidades distintas de navegación;
- adjuntar una imagen pertinente a la revisión;
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
modelo: continuar / repetir / modificar / pedir ayuda / terminar
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
| `set_arm_pose` | disminuye el error de articulaciones | pose medida dentro de tolerancia | detener el pedido de brazo sin iniciar locomoción |

Los plazos se medirán con reloj simulado cuando esté disponible. Mientras el
simulador siga lento, cada prueba declarará explícitamente si usa tiempo de
simulación o un margen transitorio de tiempo de pared.

Habrá límites duros para evitar ciclos infinitos:

- máximo 20 decisiones por misión en la primera versión;
- un reintento automático del mismo paso ante el mismo error;
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
6. Adjuntar imágenes puntuales a `look_at`, `read_clock` y `search_table`.
7. Migrar brazos y demás capacidades listas al mismo contrato.
8. ~~Mostrar en el tablero plan original, revisión, evidencia, cancelación y
   plan vigente.~~ Falta la aprobación visual de Lucas.
9. Repetir la misión con red limpia, red lenta y corte total.
10. Hacer la validación visual con Lucas.

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
6. buscará la mesa en la vista actual;
7. revisará después de cada paso si debe continuar o cambiar el plan;
8. cancelará cualquier paso trabado y quedará activamente en `STAND`;
9. mostrará la entrada, respuesta, mediciones e imagen usadas para decidir.

El sistema todavía se bloqueará honestamente si la mesa no está en la vista,
si necesita una aproximación fina o cuando llegue al agarre. Los pasos
inmediatamente posteriores serán el barrido visual activo, la aproximación a
la mesa y el agarre.

Por lo tanto, este tramo no completa aún la entrega del objeto. Convierte la
demo de una lista rígida en una misión observable que detecta fallas, se pone
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
- AGILE conserva locomoción y balance; el modelo no controla el cuerpo.
