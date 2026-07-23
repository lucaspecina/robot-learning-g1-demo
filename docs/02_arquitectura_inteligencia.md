# Arquitectura de la "inteligencia" en robots móviles (Go2 / Spot / humanoides)

> Informe de investigación (jul 2026): 25 fuentes primarias, 22 afirmaciones
> verificadas por triple voto adversarial. Pregunta: ¿cómo se arquitecta la
> inteligencia (a bordo / servidor / nube) para tareas con razonamiento —
> inspección, patrullaje, misiones en lenguaje natural?

## TL;DR

1. **El patrón dominante 2024-2026** para el "cerebro" es un agente LLM/VLM
   haciendo **tool use sobre una API de skills documentada** del robot
   (servicios/acciones ROS 2, goals de Nav2, skills nombradas tipo
   `walk_to_position`), generando código o planes estructurados en un loop
   plan → ejecutar → observar → replanear — **por encima del stack de control
   del fabricante, sin tocarlo**. Nunca comanda motores.
2. **La ubicación del cómputo se decide por latencia, memoria y ancho de
   banda**, en tres pisos: control duro y percepción rápida a bordo; modelos
   medianos y monitoreo en servidor local; razonamiento pesado en nube. Con
   números concretos (abajo).
3. **La conectividad se diseña para sobrevivir al corte**: el patrón comercial
   de referencia (Boston Dynamics) permite operar como "isla" on-premises sin
   nube; los stacks de investigación corren 100% a bordo.

## 1. El patrón dominante: agente + skills

Cuatro papers independientes (verificados 12-0) describen la misma
arquitectura:

- **maestro** (NVIDIA, 2025): Gemini Robotics-ER escribe y ejecuta código
  sobre un toolkit de percepción, planificación de movimiento, políticas VLA,
  mapa semántico, odometría lidar y navegación Nav2 — **demostrado sobre un
  Unitree Go2-W**. [arXiv 2511.00917]
- **ROS-LLM**: librería de acciones atómicas como servicios/acciones ROS con
  descripciones JSON; el LLM emite Python, secuencias JSON o behavior trees
  XML. [arXiv 2406.19741]
- **Paper de cuadrúpedos long-horizon**: 4 agentes LLM especializados
  (planner, calculador de parámetros, generador de código, replanner)
  emitiendo Python contra skills nombradas. [arXiv 2404.05291]
- **ASTIbot**: el agente LLM se conecta a un stack ROS 2 del fabricante SIN
  modificarlo, puramente en la capa de comando e interpretación; la seguridad
  queda en el stack del vendor. [MDPI Appl. Sci. 16(4):1680]

**Grounding** (cómo el modelo "ve" al robot): los LLM de texto reciben
observaciones *textualizadas* por un componente dedicado ("el gripper está
abierto"); los VLM reciben imágenes directas + estado estructurado (mapas
semánticos que cachean ubicaciones de objetos).

## 2. Dónde corre cada cosa: "decomposed execution"

El split por pisos, con ejemplos verificados:

| Piso | Qué corre | Ejemplo verificado |
|---|---|---|
| A bordo (robot/Jetson) | Control tiempo-real, locomoción, percepción rápida, SLAM/nav | Stack completo de navegación de un Spot (FAST-LIO2, NDT, FAR Planner) corriendo en una **NUC solo-CPU**, 176 ms e2e, sin red ni GPU [arXiv 2603.04470] |
| Servidor local (misma red) | LLM/VLM medianos, monitoreo rápido, fleet mgmt | maestro corre un Qwen-2.5-VL-72B local como monitor a 2 Hz; ASTIbot sirve LLMs desde una laptop con llama.cpp en la LAN, sin nube |
| Nube / remoto | Razonamiento pesado, orquestador, verificación | Gemini Robotics: backbone VLA en nube (<160 ms) + decoder local; ROS-LLM corrió el razonador intercontinental (~2-3 s de lag) sin problema |

La lección estructural: **cuanto más abajo, menos tolera latencia; cuanto más
arriba, más piensa por evento y menos por milisegundo** — el piso de razonamiento
tolera segundos.

## 3. Los números que deciden (estudio de medición Berkeley+MSR, mar 2026)

[arXiv 2603.18284], todo verificado contra el texto primario:

- **Memoria**: el stack moderno completo (mapa semántico + planning + VLA +
  navegación) necesita **~50 GB** — no entra en una Jetson Orin AGX de 32 GB.
  Algo se descarga a servidor o se cuantiza fuerte.
- **Latencia**: decenas de milisegundos extra de red degradan la precisión de
  manipulación **>10%** (π-0.5: 80%→70% de éxito con ~10 ms extra). Los loops
  finos no salen del robot o de un edge pegado.
- **Ancho de banda**: descargar la inferencia VLA con 3 cámaras 640×480@30fps
  pide **~100 Mbps** de subida sin pérdida (H.264 lo baja a 6.5-12 Mbps pero
  cuesta ~20% de precisión).
- **Energía**: GPUs grandes a bordo drenan la batería hasta 160% más rápido.
- **Conclusión del estudio**: no hay ubicación óptima universal — es un
  trade-off por caso de uso.

## 4. Conectividad: diseñar para el corte

- **El patrón comercial de referencia** (white paper de seguridad de Boston
  Dynamics): Spot + servidor Site Hub con Orbit pueden operar como **"isla"
  totalmente aislada** dentro de la red del cliente, sin conectividad externa;
  el Orbit on-prem es esencialmente el mismo software que el de nube. La nube
  es *opcional por diseño*.
- Los stacks de investigación de campo corren enteros a bordo (cero
  dependencia de red).
- **Patrón híbrido elegante, en NUESTRO hardware exacto**: un paper 2025
  desplegó *speculative decoding* edge-nube en un **Go2 EDU con Jetson Orin
  16 GB**: Qwen-2-VL-2B cuantizado a bordo como draft + Qwen-2-VL-7B en nube
  como verificador → 21% más rápido que nube sola, y degradación elegante si
  la red anda mal. [arXiv 2505.21594]
- Frameworks de partición adaptativa robot↔servidor existen sobre ROS/ROS2
  (ElasticROS, FogROS2) — partición a nivel de algoritmo. [arXiv 2209.01774]

## 5. Receta para nuestro lab (Go2 EDU + Jetson, futuro G1)

Mapeando la evidencia a nuestro caso:

1. **A bordo (Jetson)**: locomoción (sport mode o policy propia), SLAM/nav,
   YOLO y pipelines de percepción específicos (ej. lector de medidores),
   y opcionalmente un VLM chico cuantizado (2-8B) para comprensión básica
   sin red — exactamente lo que hizo el paper del Go2 EDU.
2. **Servidor local (PC con GPU del lab)**: VLM mediano para monitoreo/
   verificación, dashboards, grabación de datos. La LAN da ~1 ms.
3. **Nube (API)**: el orquestador/razonador grande, que decide por evento.
4. **Interfaz entre cerebro y robot**: una librería de skills documentada
   (el patrón universal) — nuestras "tools": navegar a waypoint, leer
   medidor X, capturar imagen, reportar. El agente compone; jamás toca abajo.
5. **Regla de resiliencia**: la misión debe poder completarse (o abortar con
   dignidad) con la nube caída — la inteligencia crítica del ciclo vive de la
   LAN para abajo.

## 6. Qué NO cubrió esta investigación (honestidad)

- Nada verificado sobre ANYmal en inspección, el pipeline específico de
  lectura de medidores (detección + keypoints) en despliegues comerciales,
  Scout/Orbit como workflow de inspección, ni el G1 en particular.
- Conectividad parcial: no hay evidencia verificada sobre 5G, VPNs concretas
  ni comportamiento de failover a mitad de misión.
- Los números del §3 salen de UN estudio (riguroso, pero no replicado) y
  dependen de su composición de workloads — no generalizar al dígito.
- Afirmaciones refutadas en verificación (NO citar): que 4 GB de VRAM sea un
  umbral duro para LLMs edge; que las minas "obliguen" a diseño 100% a bordo;
  que ElasticROS tenga garantías probadas de aprendizaje online.
- Posible sesgo de publicación: todos los ejemplos supervivientes son estilo
  agente-LLM; en producción las state machines siguen siendo comunes pero
  se publican menos.

## Preguntas abiertas (para futuras investigaciones)

1. ¿Cómo implementan los despliegues comerciales la lectura de medidores —
   keypoints a bordo o imagen al servidor, y con qué precisión?
2. ¿Qué hacen exactamente los sistemas de producción cuando se corta la
   conectividad a mitad de misión (seguir / safe-stop / volver al dock)?
3. ¿Dónde está hoy la frontera práctica de LLM/VLM a bordo en Orin (2-8B
   cuantizados) y la colapsa la Jetson Thor (128 GB)?
4. ¿Siguen siendo behavior trees / state machines la capa ejecutable "de
   registro" en productos comerciales, con el LLM confinado a generar planes?

## Fuentes principales

- maestro (NVIDIA): https://arxiv.org/html/2511.00917v1
- ROS-LLM: https://arxiv.org/html/2406.19741v1
- Cuadrúpedos long-horizon multi-agente: https://arxiv.org/pdf/2404.05291
- ASTIbot: https://www.mdpi.com/2076-3417/16/4/1680
- Estudio de medición offload (Berkeley+MSR): https://arxiv.org/html/2603.18284
- Speculative decoding en Go2 EDU: https://arxiv.org/html/2505.21594v1
- Spot en minas, stack CPU-only: https://arxiv.org/html/2603.04470v1
- Survey foundation models embodied: https://arxiv.org/html/2603.16952v1
- Cloud robotics survey: https://arxiv.org/pdf/2104.14270
- ElasticROS: https://arxiv.org/pdf/2209.01774
- BD Spot/Site Hub security white paper: https://bostondynamics.com/wp-content/uploads/2024/03/spot-and-site-hub-security-white-paper.pdf
- Stack autonomía Go2 (CMU): https://github.com/jizhang-cmu/autonomy_stack_go2
