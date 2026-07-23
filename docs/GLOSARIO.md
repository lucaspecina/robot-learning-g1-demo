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

## Locomoción (caminar)

- **Marcha (gait)**: el patrón rítmico de qué pata se levanta cuándo. Los
  cuadrúpedos tienen varias (como caballo: paso, trote, galope). El Go2 usa
  casi siempre **trote**: las patas se mueven en pares diagonales — delantera
  derecha + trasera izquierda juntas, después el otro par. Unas 2-3 zancadas
  por segundo.
- **Apoyo / vuelo (stance / swing)**: las dos fases de cada pata dentro del
  ciclo. Apoyo = el pie está en el piso empujando. Vuelo = el pie está en el
  aire viajando hacia su próximo lugar de pisada.
- **Centro de masa**: el punto donde se concentra "el peso promedio" del
  cuerpo. Para el equilibrio, lo que importa es dónde cae su vertical
  respecto de los apoyos.
- **Polígono de soporte**: la figura que dibujan en el piso los pies apoyados.
  Con 4 pies: un rectángulo. Con 2 pies (trote): apenas una línea. Regla de
  oro: si la vertical del centro de masa cae dentro del polígono, el robot se
  sostiene solo (estabilidad estática); si cae afuera, se está cayendo y
  alguien tiene que hacer algo.
- **Comando de velocidad (vx, vy, vyaw)**: la orden estándar que recibe la
  capa de locomoción: velocidad hacia adelante, hacia el costado, y de giro.
  Nota: no le decís adónde ir ni qué pata mover — solo a qué velocidad
  querés que se traslade el cuerpo.
- **Locomoción ciega / perceptiva**: ciega = caminar solo con propiocepción
  (IMU, articulaciones, pies), sin ver el terreno — reacciona al pisar.
  Perceptiva = además usa cámara/lidar para ver el terreno y elegir dónde
  pisar antes de llegar. La ciega resuelve muchísimo más de lo que uno
  esperaría; la perceptiva hace falta para escaleras rápidas, huecos,
  pisadas de precisión.

- **Gravedad proyectada**: la forma estándar de meter "cómo estoy inclinado"
  en la observación de una policy: el vector de la gravedad expresado en el
  marco del cuerpo (3 números). Derecho = (0, 0, −1); inclinado hacia
  adelante, la componente x crece. Se prefiere a roll/pitch/yaw porque es
  suave, no tiene discontinuidades, e ignora el yaw (mirar al norte o al sur
  no cambia nada para el equilibrio).

## Conectividad (cómo se habla todo)

- **Bit y byte (las unidades de datos)**: el bit es un 0 o un 1 — el átomo de
  la información. Un byte = 8 bits ≈ una letra de texto. La escalera: KB (mil
  bytes ≈ media página de texto), MB (un millón ≈ una foto), GB (mil millones
  ≈ una película). Anclas útiles: una frase ≈ 100 B · una foto ≈ 3 MB · el
  modelo Whisper small ≈ 500 MB · un VLM de 7B cuantizado ≈ 4-5 GB.
- **Mbps vs MB/s (la trampa clásica)**: las velocidades de red se miden en
  megaBITS por segundo (Mbps); los archivos en megaBYTES (MB). Como 1 byte =
  8 bits, para pasar de la velocidad de red a "cuántos MB por segundo bajo",
  dividí por 8: 100 Mbps ≈ 12.5 MB/s.

- **IP / subred**: la IP es la dirección de una compu en una red ("casa número
  161"); la subred es la calle (192.168.123.x = todas las casas que empiezan
  igual). Dos dispositivos se hablan directo solo si están en la misma calle.
  El Go2 vive en la calle 192.168.123.x: el robot es .161, tu PC debe ponerse
  .99, la Jetson de expansión suele ser .18.
- **Switch**: un "zapatillón" de red: aparatito con varios enchufes ethernet
  donde todo lo enchufado queda en la misma calle y se puede hablar entre sí.
  El Go2 tiene uno ADENTRO del cuerpo — el puerto RJ45 de la panza es
  simplemente un enchufe libre de ese switch interno.
- **DHCP**: el "recepcionista" que reparte direcciones IP automáticamente.
  Lo tienen los routers. Un cable directo PC↔robot NO tiene recepcionista —
  por eso ahí tu IP la configurás a mano (estática).
- **Ethernet (RJ45)**: red por cable. Rápida (1 Gbps), latencia bajísima
  (<1 ms) y estable. La única opción seria para control de bajo nivel a
  500 Hz. El Go2 tiene un puerto RJ45 para esto.
- **AP vs station (WiFi)**: un dispositivo WiFi puede CREAR su propia red
  (modo AP/hotspot — lo que hace el Go2 de fábrica para hablar con la app) o
  UNIRSE a una red existente (modo station). WiFi sirve para telemetría,
  video y comandos de alto nivel; para el loop de 500 Hz es ruleta (picos de
  latencia y cortes).
- **Puerto**: si la IP es la dirección del edificio, el puerto es el número
  de departamento. Una misma compu puede tener muchos programas atendiendo a
  la vez, cada uno en su puerto (un número: 8000, 8080...). "Mandar al
  servidor" siempre significa, concretamente: mandar a IP:puerto, donde un
  programa específico está escuchando.
- **SSH**: terminal remota — abrís una consola de otra compu desde la tuya,
  por la red. Es como se trabaja "adentro" de la Jetson sin conectarle
  teclado ni monitor.
- **Streaming de video**: la cámara no "manda fotos": comprime el video
  (H.264) y lo emite como un mini-Twitch privado (~2-8 Mbps) que tu pantalla
  decodifica. El Go2 lo hace vía GStreamer/WebRTC.
- **VPN**: túnel cifrado por internet que te "teletransporta" a una red
  lejana: tu notebook en tu casa aparece como una casa más en la calle del
  lab, y podés hablar con el robot como si estuvieras ahí.
- **Latencia vs ancho de banda**: cuánto TARDA un mensaje vs cuánto VOLUMEN
  por segundo entra. La escalera típica: cable <1 ms · WiFi 2-20 ms (con
  picos) · servidor en la misma red ~1 ms · nube 50-200 ms. Esta escalera es
  la que decide qué parte de la inteligencia puede vivir dónde.

## Navegación (ir a lugares)

- **Teleoperación (teleop)**: un humano mandando los comandos de velocidad en
  vivo — joystick, gamepad o teclado. La capa de navegación más simple: el
  cerebro sos vos.
- **Odometría**: estimar dónde estás acumulando pistas del propio movimiento
  (cuánto giró cada pata, qué dice la IMU). Funciona en el corto plazo pero
  el error se acumula: tras 20 metros podés estar 1 metro desviado. Es lo
  que publica rt/sportmodestate en el robot real.
- **SLAM**: construir un mapa del entorno con lidar/cámara y ubicarte dentro
  de él, al mismo tiempo. A diferencia de la odometría, no acumula deriva:
  reconocer un lugar ya visto corrige la posición. Es lo que hace falta para
  "andá al punto (3, 2)" de verdad y sin trampa.

## Entrenar policies

- **Domain randomization**: entrenar con la física deliberadamente variada
  entre episodios — fricción del piso, masa del robot, potencia de motores,
  retardos de comunicación, empujones aleatorios. Objetivo: que la policy no
  memorice UN simulador sino que sea robusta a una familia de mundos, entre
  los cuales (con suerte) está el real.
- **Sim2sim / sim2real**: las dos validaciones del pipeline. Sim2sim = correr
  la policy entrenada en el simulador A en un simulador B distinto (p. ej.
  entrenada en Isaac Gym, validada en MuJoCo): si solo funciona en A,
  aprendió trucos del motor de física, no a caminar. Sim2real = el paso
  final al robot físico.
- **rsl_rl**: la librería de PPO que usa todo el ecosistema legged (la hizo
  el lab de ETH que creó ANYmal). Optimizada para miles de entornos
  paralelos en GPU.

## Las capas de arriba (aún no las usamos)

- **Política (policy)**: en RL, la función que decide acciones a partir de
  observaciones. En locomoción: entra el lowstate (+ el comando de velocidad
  deseado), salen los 12 objetivos de posición para el PD, ~50 veces/s. Una
  política entrenada ocupa el lugar exacto de la rampa tanh del stand.
- **Observación (obs)**: el vector concreto que entra a la política en cada
  paso. Para locomoción ciega del Go2, ~45 números: orientación y velocidad
  angular del cuerpo (IMU), las 12 q y 12 dq, la acción del paso anterior y
  el comando de velocidad. Todo sale de rt/lowstate + el comando.
- **Pose nominal / action scale**: truco estándar de las políticas de
  locomoción: la red no emite ángulos absolutos sino desvíos chicos respecto
  de una pose de referencia (la de parado): `q_des = pose_nominal +
  escala × acción`. Arranca "cerca de bien" y el espacio de acciones queda
  acotado y centrado.
- **Model-free / model-based**: cuidado con la palabra "modelo", que en esta
  zona significa dos cosas distintas. (1) El modelo-red-neuronal: los pesos
  de la política. (2) El modelo-del-mundo: las ecuaciones físicas del robot.
  Las políticas de locomoción típicas se entrenan model-free (PPO): no
  aprenden ni usan un modelo del mundo explícito — la física existe solo
  adentro del simulador durante el entrenamiento. MPC es lo contrario:
  model-based puro, usa las ecuaciones online y no aprende nada.
- **MPC (control predictivo)**: la alternativa clásica a RL para la misma capa:
  con un modelo físico del robot, optimizar numéricamente el movimiento de los
  próximos ~0.5 s, ejecutar solo el primer pasito, y volver a optimizar —
  decenas de veces por segundo.
- **Sport mode**: el controlador de fábrica del Go2 (su capa de locomoción +
  equilibrio, corriendo a bordo). Se usa vía SportClient (`Move(vx, vy, vyaw)`,
  `StandUp()`...). Para comandar motores directo por `rt/lowcmd` hay que
  apagarlo primero — si no, dos controladores pelean por los mismos motores.
  No existe en el simulador.
- **VLM (vision-language model)**: modelo grande que recibe imágenes + texto y
  responde texto (GPT-4V, Claude, Qwen-VL...). Entiende escenas en abierto
  ("¿qué se ve acá? ¿hay algo fuera de lugar?") pero es poco confiable para
  medir con precisión (leer una aguja, medir distancias). No mueve nada: para
  que "actúe" hay que darle herramientas a las que llamar.
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
