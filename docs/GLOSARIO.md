# Glosario

Términos que fueron apareciendo en el camino, explicados en simple. Referencia
acumulativa: se agrega uno nuevo cada vez que aparece en las conversaciones o
el código. Si un término de acá abajo no se entiende, es un bug de este
documento — corregirlo.

## El cuerpo del robot

- **Articulación (joint)**: un punto del cuerpo que puede girar, como tu codo o
  rodilla. El Go2 tiene 12 con motor: 3 por pata (hip = abre la pata hacia el
  costado, thigh = muslo adelante/atrás, calf = rodilla).
- **q / dq**: la posición de una articulación (cuán doblada está, en radianes) y
  su velocidad (cuán rápido está girando, en radianes por segundo). La notación
  viene de física; `d` por "derivada".
- **Radián (rad)**: unidad de ángulo. 1 rad ≈ 57°; 1.57 rad = 90°; π rad = 180°.
  Todo el mundo robótico mide ángulos así, no en grados.
- **Torque**: fuerza de rotación, medida en newton-metro (N·m). Es lo ÚNICO que
  un motor puede hacer: retorcer su articulación con más o menos fuerza, hacia
  un lado o el otro. Torque constante no significa movimiento constante — si
  algo devuelve la misma fuerza en contra (el piso, un tope mecánico, la
  gravedad), el resultado es quietud con esfuerzo.
- **El cuerpo (base/torso)**: la caja central del robot, de la que salen las 4
  patas. Importante: los 12 ángulos de las articulaciones NO te dicen cómo está
  el cuerpo — un robot con las patas en pose de parado puede estar de pie o
  patas arriba con los mismos 12 ángulos.

## Sensores

- **IMU**: el chip sensor de equilibrio del cuerpo — el mismo que tu teléfono
  usa para rotar la pantalla. Va montado en el torso y responde dos preguntas:
  ¿cómo estoy inclinado? (orientación) y ¿qué tan rápido estoy rotando?
  (velocidad angular). Es el equivalente del oído interno. OJO: no sabe DÓNDE
  está el robot en la habitación, solo cómo está orientado.
- **Orientación (roll / pitch / yaw)**: cómo está inclinado el cuerpo respecto
  de la vertical, en tres ángulos. Roll = rolar de costado (un ala sube, la
  otra baja). Pitch = trompa arriba / trompa abajo. Yaw = hacia qué punto
  cardinal apunta. Un robot bien parado en piso plano tiene roll ≈ 0 y
  pitch ≈ 0.
- **Velocidad angular (giróscopo)**: qué tan rápido está ROTANDO el cuerpo en
  este instante, en rad/s. Distinto de la orientación: podés estar horizontal
  (pitch = 0) pero rotando rápido hacia adelante — es decir, en plena caída.
  Para el equilibrio, esta señal es tan importante como la inclinación misma.
- **Acelerómetro**: la tercera lectura de la IMU: la aceleración del cuerpo
  (sacudones, frenadas, caída libre), en m/s². Quieto y derecho marca +9.81
  hacia arriba — la gravedad — y de ahí se deduce "dónde es abajo".
- **foot_force**: sensores en las plantas de los 4 pies que miden cuánta fuerza
  hace cada pie contra el piso. (El simulador Python no los simula: quedan
  en 0. El robot real sí los publica.)
- **Propiocepción / exterocepción**: los dos grandes grupos de sentidos.
  Propiocepción = sentir el propio cuerpo (articulaciones, IMU, pies,
  temperatura, batería) — todo lo que hay en `rt/lowstate`. Exterocepción =
  sentir el mundo exterior (cámara, lidar, micrófono). La locomoción básica
  se resuelve solo con propiocepción; ver el mundo es capa aparte.

## Comunicación

- **DDS**: el sistema de mensajería del robot. Pub/sub sin servidor central:
  cada programa anuncia qué publica o escucha, y se conectan directo entre sí.
  El mismo esquema conceptual que Kafka, sin broker.
- **Topic**: un "canal" o buzón de mensajes con nombre, p. ej. `rt/lowstate`.
- **rt/lowstate**: el topic donde el robot cuenta su estado ~500 veces/s (200
  en el sim Python): q, dq y torque de cada motor, IMU, pies, batería. Solo
  lectura: el robot habla, vos escuchás.
- **rt/lowcmd**: el topic donde vos le das órdenes de bajo nivel a los motores:
  por cada motor, un objetivo de posición + kp + kd (+ torque extra opcional).
- **rt/sportmodestate**: topic donde el controlador de fábrica publica su
  estimación de la situación del cuerpo: posición y velocidad estimadas,
  posiciones de los pies, tipo de marcha. En el sim es "ground truth" (el
  simulador sabe la verdad exacta y la regala); en el robot real solo existe
  con el sport mode encendido, y es una estimación.
- **Domain ID**: número que aísla mundos DDS: domain 1 = simulador, domain 0 =
  robot real. Programas en domains distintos no se ven entre sí.
- **CRC**: sello de integridad al final de cada mensaje de comando — un
  resumen matemático del contenido. El robot real descarta mensajes cuyo CRC
  no cierra (protección contra datos corruptos); el sim ni lo mira.

## Control

- **Control PD**: la regla que convierte "quiero la articulación en el ángulo
  X" en torque: `torque = kp·(objetivo − posición) + kd·(0 − velocidad)`.
  La calcula el chip de cada motor miles de veces por segundo (en sim, el
  bridge). Primer término: empuja hacia el objetivo, proporcional al error.
  Segundo: frena la velocidad para no oscilar. El efecto combinado es el de
  un resorte con amortiguador entre la pata y el objetivo.
- **kp / kd**: las dos perillas del control PD. kp = rigidez (cuánto torque por
  radián de error; alto = duro, 0 = flojo). kd = amortiguación (cuánto frena
  el movimiento). En el stand usamos kp=50, kd=3.5; con kp=0 y kd=2 el robot
  queda "blando" (estado seguro).
- **Error (de posición)**: la distancia entre donde está la articulación y
  donde el objetivo dice que debería estar. El torque del PD es proporcional
  a esto — por eso mantener el error chico (moviendo el objetivo de a poco)
  produce movimientos suaves.
- **Interpolación**: generar los puntos intermedios entre una pose y otra:
  `objetivo(t) = (1−s)·pose_A + s·pose_B` con `s` subiendo de 0 a 1. Es cómo
  el stand desliza el objetivo en vez de saltarlo.
- **Lazo abierto / lazo cerrado**: un control es de lazo cerrado si mide el
  resultado y corrige sobre la marcha; de lazo abierto si ejecuta su plan a
  ciegas. El stand es lazo abierto a nivel trayectoria (reproduce una
  secuencia fija) pero cada articulación cierra su lazo abajo (el PD usa la
  posición medida en cada instante).
- **Pose**: una "foto" de los 12 ángulos articulares. Pose de parado, pose
  agachada, etc.

## Las capas de arriba (aún no las usamos)

- **Política (policy)**: en RL, la función que decide acciones a partir de
  observaciones. En locomoción: entra el lowstate (+ el comando de velocidad
  deseado), salen los 12 objetivos de posición para el PD, ~50 veces/s. Una
  política entrenada ocupa el lugar exacto de la rampa tanh del stand.
- **MPC (control predictivo)**: la alternativa clásica a RL para la misma capa:
  con un modelo físico del robot, optimizar numéricamente el movimiento de los
  próximos ~0.5 s, ejecutar solo el primer pasito, y volver a optimizar —
  decenas de veces por segundo.
- **Sport mode**: el controlador de fábrica del Go2 (su capa de locomoción +
  equilibrio, corriendo a bordo). Se usa vía SportClient (`Move(vx, vy, vyaw)`,
  `StandUp()`...). Para comandar motores directo por `rt/lowcmd` hay que
  apagarlo primero — si no, dos controladores pelean por los mismos motores.
  No existe en el simulador.
- **VLA (vision-language-action)**: modelo grande que recibe cámara + una
  instrucción en lenguaje y emite directamente acciones: típicamente un
  "action chunk" — una ráfaga de objetivos de posición para las
  articulaciones del brazo/manos (p. ej. el próximo segundo de trayectoria,
  50 objetivos), que abajo ejecuta el control PD de siempre. Para
  manipulación (brazos), el VLA fusiona en una sola red el "qué hacer" y el
  "generar objetivos". Para equilibrio/locomoción NO: ahí una política
  rápida (RL) conserva el asiento del medio y el VLA le pasa comandos desde
  arriba. Lo que nunca reemplaza: la capa PD/torques.
- **Action chunk**: la ráfaga de acciones que un VLA emite por inferencia — una
  mini-trayectoria precalculada (~0.5–1 s de objetivos) que se ejecuta en lazo
  abierto mientras el modelo, lento, prepara la siguiente. El mismo patrón que
  la rampa del stand: planear un tramo corto, reproducirlo, replanear.
