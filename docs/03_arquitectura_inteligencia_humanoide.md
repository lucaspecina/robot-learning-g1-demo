# Arquitectura de inteligencia para un humanoide, acotada por el cómputo real

> Investigación de agosto de 2026. Continúa
> [`02_arquitectura_inteligencia.md`](02_arquitectura_inteligencia.md), que hizo
> el mismo ejercicio para robots móviles en julio de 2025. La diferencia de
> este documento es la restricción: **no se pregunta qué arquitectura es mejor,
> sino cuál entra en la computadora que el robot lleva encima, qué latencia
> tolera cada capa y qué pasa cuando se corta la red.**

## TL;DR

1. **El G1 tiene dos computadoras.** La locomoción corre en una cerrada de
   Unitree; la nuestra es una Jetson Orin NX de 16 GB. Eso es una buena noticia
   —no gastamos cómputo en caminar— y una mala: 16 GB es exactamente el mínimo
   que pide un VLA de 3 B, antes del sistema operativo.
2. **NVIDIA, en su propio flujo oficial para el G1, exige una Jetson AGX Thor
   externa.** Si el Orin NX alcanzara, la usarían. Es una decisión de compra, no
   de software.
3. **Aun sobre Thor, GR00T corre a 5 Hz** en el paquete de referencia de NVIDIA.
   No a 40. El cuello de botella es el modelo de visión, no la placa.
4. **La arquitectura modular que ya tenemos es la que Boston Dynamics y Google
   acaban de publicar como patrón de referencia.** No es un escalón provisorio.
5. **La decisión de diseño más barata y de mayor retorno es ejecutar acciones en
   bloques asincrónicos.** Con eso, 200 ms de red salen gratis; sin eso, 100 ms
   cuestan el 40 % del rendimiento.
6. **Afinar un modelo de manipulación cuesta menos de 20 dólares. Las 200
   demostraciones que necesita, no.**

---

## 1. El hardware manda

### 1.1 Las dos computadoras del G1

| Computadora | Dirección interna | Qué hace | ¿Nuestra? |
|---|---|---|---|
| Locomoción | `192.168.123.161` | control de motores, controlador propietario | **no**, cerrada |
| Desarrollo | `192.168.123.164` | Jetson Orin NX sobre placa de Unitree | sí |

Fuente: [guía de desarrollo del G1 de Weston Robot](https://docs.westonrobot.com/tutorial/unitree/g1_dev_guide/),
distribuidor oficial. Advierten explícitamente que no se debe reinstalar el
sistema del módulo Orin con imágenes de terceros: hay que usar el de Unitree.

Esto explica y confirma lo que vimos en el código de Humandroid: su nodo de
odometría se suscribe a `/lf/odommodestate`, o sea consume lo que publica la
computadora cerrada en vez de estimar movimiento por su cuenta.

La página oficial de Unitree es deliberadamente vaga sobre el módulo — dice
“CPU de 8 núcleos” y “varias marcas y modelos disponibles, como Orin NX”. Los
distribuidores especifican **Orin NX de 16 GB**.

### 1.2 Qué es realmente un Orin NX de 16 GB

- **100 TOPS INT8 “sparse” / 50 TOPS “dense”.** El número de folleto es el
  primero; el segundo es el que corresponde a un modelo normal.
- 16 GB de memoria **compartida entre CPU y GPU** — no hay memoria de video
  aparte —, con 102,4 GB/s de ancho de banda.
- 10 a 25 W configurables.

Fuente: [hoja de datos de NVIDIA](https://developer.nvidia.com/downloads/jetson-orin-nx-series-data-sheet).

### 1.3 Por qué eso no alcanza para un VLA

Un VLA es un modelo que recibe imagen, instrucción en texto y estado del robot,
y devuelve movimientos. El de referencia de NVIDIA, **GR00T N1.7, tiene 3.000
millones de parámetros ≈ 6 GB sólo de pesos**, y su repositorio pide **una GPU
con 16 GB o más** para inferencia.

Ese mínimo es *toda* la memoria del robot, antes de Ubuntu, ROS 2, Nav2, SLAM,
los detectores y los drivers de las cámaras.

Y no hace falta deducirlo: el
[flujo oficial de NVIDIA para el G1](https://docs.nvidia.com/learning/physical-ai/gr00t-e2e-workflow/latest/real-robot-workflow/real-overview.html)
describe la configuración como *G1 con manos Dex3-1, una cámara RealSense en la
cabeza y **una Jetson AGX Thor** conectada externamente o por la mochila de
Unitree*. Si el Orin NX alcanzara, no pondrían otra computadora encima.

### 1.4 La Thor, y lo que nadie publicó

| | Orin NX 16GB (G1 de fábrica) | AGX Thor |
|---|---|---|
| Memoria | 16 GB | **128 GB** |
| Ancho de banda | 102,4 GB/s | 273 GB/s |
| Potencia | 10–25 W | **40–130 W** |
| Precio del kit | ~500–700 USD | **3.499 USD** |

La mochila que la monta en el G1 se vende por unos **15.200 USD** en
revendedores y estaba **agotada** al consultar. **No pudimos confirmar precio ni
disponibilidad en fuente primaria de Unitree.**

Dos riesgos que nadie documentó y que conviene preguntar antes de comprar:

**Autonomía.** El G1 lleva una batería de 421 Wh con unas 2 h declaradas, lo que
da unos 210 W promedio. Una Thor consume entre 40 y 130 W, o sea **entre el 19 %
y el 62 % del presupuesto energético actual**. A consumo alto, la autonomía
caería a poco más de una hora.

**Calor.** No hay ningún dato publicado de disipación de una Thor montada en un
robot cerrado y en movimiento.

---

## 2. Las tres familias de arquitectura

### 2.1 Un solo modelo que hace todo

El ejemplo con más sustancia es **Helix 02 de Figure**
([anuncio](https://www.figure.ai/news/helix-02)). La prensa lo describió como
“una sola red”, pero el texto describe tres capas dentro de un mismo sistema
entrenado: una de 10 millones de parámetros a **1 kHz** que reemplaza a “109.504
líneas de C++ escrito a mano”, otra visuomotora a **200 Hz**, y una de
razonamiento más lenta.

Se entrenó con **más de 1.000 horas de movimiento humano** trasladado a las
articulaciones del robot y **más de 200.000 entornos simulados en paralelo**.

Lo que publican como resultado es **una tarea de cuatro minutos con 61 acciones
encadenadas**. No publican tasa de éxito, ni parámetros, ni qué placa llevan
a bordo. Es un video largo sin denominador.

El **UnifoLM-OmniA de Unitree**, que apareció en nuestra investigación anterior,
está peor documentado todavía: **no hay repositorio, ni pesos, ni paper, ni
métricas.** Lo que Unitree sí liberó son piezas anteriores —un modelo de mundo y
un VLA sobre Qwen2.5-VL— pero **con licencia no comercial**: sirven para
aprender, no para desplegar en una refinería.

### 2.2 Razonador arriba, ejecutor abajo

Es la familia con más evidencia publicada.

**Gemini Robotics** separa explícitamente un modelo de razonamiento espacial
—que corre en la nube por API— de un VLA que ejecuta localmente. A agosto de
2026 el razonador está disponible como `gemini-robotics-er-2-preview`; **el VLA
sigue cerrado a socios seleccionados**, igual que la versión que corre a bordo.

Un número muy relevante para una refinería: la versión anterior del razonador
**lee instrumentos industriales con 93 % de acierto**, contra 23 % de la
anterior — manómetros circulares, indicadores de nivel, displays digitales,
visores de líquido
([DeepMind](https://deepmind.google/blog/gemini-robotics-er-1-6/)).

**GR00T N1.7 de NVIDIA** mete los dos niveles dentro de un mismo modelo de 3 B:
un modelo de visión y lenguaje que interpreta, y un generador de acciones que
convierte esa interpretación en comandos. Se preentrenó con **20.854 horas de
video egocéntrico humano** y está validado sobre el Unitree G1.
**Su licencia permite uso comercial**, a diferencia del de Unitree.

### 2.3 Agente con capacidades declaradas — la que ya tenemos

Boston Dynamics y Google publicaron esta arquitectura como patrón de referencia
en
[Tools for Your To Do List with Spot and Gemini Robotics](https://bostondynamics.com/blog/tools-for-your-to-do-list-with-spot-and-gemini-robotics/).

El modelo actúa como un operador humano tomando decisiones de alto nivel,
mientras **el software existente del robot maneja locomoción, navegación y
manipulación**. Las capacidades expuestas son guiones livianos que traducen la
decisión del modelo en llamadas a la interfaz del robot, cada uno con respuesta
en lenguaje (“levanté el objeto”, “no puedo agarrar con la mano ocupada”).

Y la frase que define la familia:

> *“Gemini Robotics tiene límites estrictos en este escenario. **No puede
> inventar capacidades nuevas ni controlar a Spot más allá de lo disponible en
> la interfaz. Esto mantiene el comportamiento de Spot predecible.**”*

Es, palabra por palabra, el principio que ya está implementado en
`skill_catalog.py` y en la doble validación de `agent/agent.py`.

Boston Dynamics aclara que es *“un paso experimental, no una aplicación
endurecida”* y no publica métricas.

### 2.4 Comparación por modo de falla

| | Monolítico | Jerárquico | Modular con capacidades |
|---|---|---|---|
| Qué corre a bordo | todo | el ejecutor | todo salvo el razonador |
| Qué se entrena | todo | el ejecutor | **nada** |
| Sin red | sigue igual | termina el bloque en curso y se queda sin objetivos | conserva navegación, equilibrio y seguridad; deja de aceptar tareas nuevas |
| Auditable | no | parcialmente | sí |

La columna del medio importa más de lo que parece: **es la única de las tres que
degrada suave**, y es la que ya tenemos.

---

## 3. Los números que deciden

### 3.1 A cuántos cuadros por segundo corre un VLA

Mediciones oficiales de NVIDIA para GR00T N1.7 (3 B, una cámara), del
[repositorio de despliegue](https://github.com/NVIDIA/Isaac-GR00T/blob/main/scripts/deployment/README.md):

| Hardware | Sin optimizar | Optimizado |
|---|---|---|
| H100 80GB | 86 ms | **35,9 Hz** |
| RTX 4090 | 82 ms | 23,3 Hz |
| **Jetson AGX Thor** | 117 ms | **10,7–10,9 Hz** |
| Jetson AGX Orin 64GB | 300 ms | 4,6–5,8 Hz |

Tres advertencias sobre esta tabla:

1. **La Thor no da el salto que promete el folleto.** Su ganancia por
   optimización es la peor de toda la tabla (1,3×), porque el cuello de botella
   es el modelo de visión, no la aritmética.
2. **El “Orin” de esa fila es un AGX Orin de 64 GB**, no el Orin NX de 16 GB del
   G1. Para nuestro robot de fábrica, esos 4,6 Hz son un techo inalcanzable.
3. Y el número que de verdad importa: el
   [paquete oficial de NVIDIA para el G1](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_physical_ai/isaac_ros_unitree_g1_gr00t/index.html)
   trae **5 Hz por defecto**, hasta unos 10 según margen de GPU. Sobre una Thor
   de 3.499 dólares.

Para contexto, otros modelos: **π₀** ocupa 14 GB en inferencia; **OpenVLA** son
7 B y corre a ~6 Hz en una RTX 4090; **SmolVLA** son 450 millones de parámetros
y ~2 GB, y es **el único que entraría cómodo en el Orin NX del G1** —a costa de
bastante menos capacidad.

### 3.2 Presupuesto de latencia por capa

| Capa | Frecuencia |
|---|---|
| Equilibrio / torque | **1 kHz** |
| Control de cuerpo completo | **200 Hz**, plazo duro |
| VLA de manipulación | 10–35 Hz a bordo |
| Razonamiento semántico | de 7 Hz a segundos |

Están separadas por **tres órdenes de magnitud**, y esa separación no es
estética. La documentación de NVIDIA lo dice sin rodeos:

> *“Cuando GR00T corre junto a la política de equilibrio AGILE en tiempo real
> (200 Hz), ambos compiten por la GPU. **Sin aislamiento, la inferencia de GR00T
> mata de hambre a AGILE, causando sobrepasos del controlador y la caída del
> robot.**”*

La solución que traen en su propio guión de arranque es reservarle a la política
de equilibrio un porcentaje fijo de la GPU. O sea: **el robot se cae si el
modelo grande le come el procesador gráfico.** No es teoría, es configuración
obligatoria.

### 3.3 Cuánto cuesta la latencia de red — el número clave

Physical Intelligence lo midió directamente
([arXiv:2506.07339](https://arxiv.org/abs/2506.07339)). Tarea: doblar ropa.

| Latencia agregada | Ejecución ingenua | Con bloques asincrónicos |
|---|---|---|
| +0 ms | 100 % | 100 % |
| **+100 ms** | **60 %** | **100 %** |
| **+200 ms** | **30 %** | **100 %** |

El mecanismo es simple de explicar: en vez de pedir una acción, ejecutarla y
volver a pedir, el robot **pide un bloque de acciones y genera el siguiente
mientras ejecuta el actual**, empalmándolos sin cortes. No requiere reentrenar
nada. Sin eso, en el borde entre bloques *“cambiar al bloque nuevo termina en
desastre”* — discontinuidades y aceleraciones peligrosas.

El equivalente abierto es la inferencia asincrónica de LeRobot, que reporta
**el doble de tareas completadas** que el modo sincrónico.

**Conclusión: descargar la inferencia a un servidor es viable, pero sólo con
ejecución en bloques asincrónicos.**

### 3.4 Ancho de banda

No hay mediciones primarias publicadas. Las cuentas propias dan el orden de
magnitud: mandar dos cámaras crudas a 30 cuadros por segundo pediría unos
440 Mbps, inviable por wifi. Pero **un VLA no necesita 30 observaciones por
segundo**: con bloques de acciones alcanza con una o dos por segundo, lo que en
imágenes comprimidas son **1 a 3 Mbps**. Perfectamente viable.

**El ancho de banda no es el problema; la latencia y su variabilidad sí.**

---

## 4. Qué cuesta entrenar cada capa

### 4.1 Locomoción: no entrenar

WBC-AGILE ya está validado sim-to-real en G1, y su costo de reproducción es alto
(del orden de cuatro placas de gama alta durante un día y medio para una tarea
de locomoción típica). Es la capa con la peor relación entre lo que se aprende y
lo que cuesta rehacerla, y la de mayor riesgo físico.

Detalle del taller de entrenamiento en
[`../g1/AGILE_TRAINING_REFERENCE.md`](../g1/AGILE_TRAINING_REFERENCE.md).

### 4.2 Manipulación: afinar, no preentrenar

Preentrenar un VLA desde cero está fuera de escala por varios órdenes de
magnitud: GR00T N1.5 se entrenó con **1.000 placas H100 durante 250.000 pasos**.

Afinar uno ya entrenado, en cambio, es accesible:

| Camino | Placa mínima | Tiempo | Demostraciones |
|---|---|---|---|
| **GR00T N1.7, flujo oficial para G1** | 1 placa de 48 GB | **2–3 h** | **≥200** |
| π₀ con adaptadores livianos | ~22,5 GB (RTX 4090) | — | — |
| OpenVLA con adaptadores | 1× A100 | 10–15 h | — |

Alquilar una H100 cuesta entre 2 y 6 dólares la hora. **Un ajuste de tres horas
sale menos de 20 dólares.**

La curva honesta de cuánto rinde según cuántas demostraciones tengas:

| Demostraciones | Resultado |
|---|---|
| 20 | nunca se reportó éxito en robot real |
| **~200** | **60–70 %** en el G1 real, flujo de NVIDIA |
| 1.000 | 98,8 % en objetos conocidos, 84,2 % en objetos nuevos |

La guía de datos de NVIDIA es concreta: **al menos 200 demostraciones limpias y
exitosas**, de 200 a 400 pasos cada una, **manteniendo fijas** las variables
—objeto, altura de mesa, iluminación, posición de cámaras— salvo que se busque
generalización a propósito.

**El ajuste no es el costo. Las 200 demostraciones sí.**

### 4.3 Multiplicar demostraciones

| Herramienta | Entrada | Salida | Resultado |
|---|---|---|---|
| **HumanoidMimicGen** | **1** demostración | **1.000** | **89 % en nueve tareas industriales**; en robot real, de 51 % a 71 % |
| DexMimicGen | 60 | >20.000 | 69–97 % según tarea |
| Isaac Lab Mimic | pocas, anotadas | 1.000 | 65–80 % de generación, 18–40 min |

**HumanoidMimicGen es el hallazgo más relevante de toda la investigación para
nuestro caso**: es de NVIDIA con UT Austin, corre **sobre Unitree G1**, y su
banco de pruebas son **nueve tareas industriales** —levantar cajas, taladrar,
empujar, sortear obstáculos—. Es literalmente el caso de uso del almacén.

El costo escondido: las herramientas de generación por video piden del orden de
**80 GB de memoria de video**. Eso es alquiler de placa grande, no la VM actual.

### 4.4 Razonamiento: no se entrena

Consenso en las tres familias: el razonador es un modelo grande de terceros al
que se le dan instrucciones y capacidades. Nadie lo entrena desde cero.

---

## 5. Recomendación

### 5.1 Antes de comprar

**Si el G1 se compra para hacer manipulación aprendida, presupuestar la Jetson
Thor desde el día uno.** El Orin NX de fábrica no da, y no es cuestión de
optimizar. Si la mochila de Unitree sigue agotada, el kit de NVIDIA conectado
por Ethernet es lo que usa la propia documentación de NVIDIA.

Y preguntar por escrito las dos cosas que nadie publicó: **cuánta autonomía se
pierde y cómo se disipa el calor.**

### 5.2 El camino, de menor a mayor riesgo

> **Decisión del 10-ago-2026 que corrige lo que decía este documento.** La
> versión anterior recomendaba cambiar el modelo de razonamiento por el de
> Google. **Se descartó**: Humandroid trabaja dentro del ecosistema NVIDIA y
> meter una pieza de Google fragmenta el stack. **Nos quedamos en NVIDIA**:
> Isaac para simular, Cosmos para razonar y multiplicar datos, GR00T para
> manipular, AGILE para entrenar por refuerzo.

**Uno.** Tratar la **detección de éxito** como una capacidad de primera clase.
Es lo que convierte un catálogo de habilidades en un agente que se recupera de
sus errores. El candidato dentro del stack de NVIDIA es **Cosmos Reason como
verificador visual** — mirar el video y responder si el paso salió bien.
Disponible como API hospedada, sin necesidad de placa propia. Ojo: **no es un
planificador de misiones**, no hay ejemplos oficiales de ese uso; para armar el
plan sigue sirviendo un modelo de lenguaje general.

**Dos.** Conservar el modelo de lenguaje actual para la planificación. Es la
pieza más barata de cambiar más adelante si el catálogo de capacidades se
mantiene como interfaz estable.

**Tres.** **Una sola** habilidad de manipulación aprendida. La más simple y
repetitiva del almacén. Recolectar las 200 demostraciones, afinar GR00T N1.7 en
la nube por menos de 20 dólares, desplegar a 5 Hz sobre la Thor con la GPU
particionada. **Esperar 60–70 %, no 95 %.** Ese número es el aprendizaje.

**Cuatro.** Recién después, multiplicar demostraciones con HumanoidMimicGen o
Isaac Lab Mimic. Es la mayor palanca publicada, pero exige tener el entorno
modelado en simulación primero — que es justamente donde el gemelo digital del
almacén se vuelve útil.

### 5.3 Qué no hacer

- **Preentrenar un VLA.** Mil placas H100.
- **Reentrenar locomoción.** Ya está resuelto y validado.
- **Un monolítico tipo Helix.** Mil horas de captura de movimiento humano, y su
  modo de falla es no tener modo de falla.
- **Correr un modelo de 3 B en el Orin NX de fábrica.** Los números no dan.
- **Planificar sobre el VLA de Gemini.** Sigue cerrado.
- **Usar el VLA de Unitree en producción.** Licencia no comercial.
- **Poner el modelo de lenguaje dentro del lazo de control.** Nunca. NVIDIA
  documenta que mezclarlos tira el robot al piso.

### 5.4 Qué es reversible y qué ata

**Reversible en días**: qué modelo de razonamiento se usa, dónde corre la
inferencia, qué VLA se afina —si los datos están en formato LeRobot, que los
tres consumen—, y la política de locomoción mientras siga siendo un módulo con
interfaz de velocidad.

**Ata durante meses**: el hardware de cómputo, porque decide qué familia es
siquiera posible; **el formato y la calidad de las demostraciones**, que es el
activo más caro y el que se tira entero si se recolecta mal; colapsar la
arquitectura en un modelo único, que es camino de ida y pierde la auditabilidad
que una refinería probablemente exija; y poner el ejecutor en la nube, porque la
degradación sin red no es graciosa.

---

## 6. Lo que no se pudo confirmar

Se deja explícito para no tratarlo como sabido:

1. Precio y disponibilidad reales de la mochila Thor — sólo revendedores, y
   agotada.
2. Disipación térmica de una Thor sobre un G1 en movimiento — nadie lo publicó.
3. Margen real de procesador y memoria del Orin NX del G1 con un stack ROS 2
   completo — **no existe medición pública**. La nuestra sobre la VM es
   probablemente lo más cercano que hay.
4. Los 250 ms declarados del stack de Gemini Robotics — vienen de prensa técnica
   glosando el paper, no del paper.
5. Parámetros, placa y tasas de éxito de Helix 02 — Figure no los publica.
6. Que UnifoLM-OmniA exista como algo más que un video.
7. Frecuencia exacta del lazo de control de bajo nivel del G1 por su interfaz.
8. Anchos de banda medidos para inferencia remota — los del documento son
   cuentas propias.
9. Una afirmación de terceros de que OpenVLA corre a 5 cuadros por segundo en un
   Orin NX de 16 GB: **descartada por inconsistente** con lo medido en placas
   diez veces más potentes.
10. Tiempos de entrenamiento y puntos de control de WBC-AGILE: el repositorio no
    los publica.

## Fuentes principales

- [Isaac GR00T N1.7](https://github.com/NVIDIA/Isaac-GR00T) · [modelo](https://huggingface.co/nvidia/GR00T-N1.7-3B) · [flujo oficial con G1](https://docs.nvidia.com/learning/physical-ai/gr00t-e2e-workflow/latest/real-robot-workflow/real-overview.html) · [paquete Isaac ROS para G1](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_physical_ai/isaac_ros_unitree_g1_gr00t/index.html)
- [Boston Dynamics + Gemini Robotics](https://bostondynamics.com/blog/tools-for-your-to-do-list-with-spot-and-gemini-robotics/) · [ejemplos](https://github.com/google-gemini/robotics-samples)
- [Gemini Robotics-ER 2](https://blog.google/innovation-and-ai/models-and-research/google-deepmind/gemini-robotics-er-2/) · [docs](https://ai.google.dev/gemini-api/docs/robotics-overview) · [lectura de instrumentos](https://deepmind.google/blog/gemini-robotics-er-1-6/)
- [Figure Helix 02](https://www.figure.ai/news/helix-02) · [Helix 1](https://www.figure.ai/news/helix)
- [Real-Time Action Chunking](https://arxiv.org/abs/2506.07339) · [openpi](https://github.com/Physical-Intelligence/openpi) · [LeRobot asincrónico](https://huggingface.co/docs/lerobot/async)
- [HumanoidMimicGen](https://humanoidmimicgen.github.io/) · [DexMimicGen](https://dexmimicgen.github.io/) · [Isaac Lab Mimic](https://isaac-sim.github.io/IsaacLab/main/source/overview/imitation-learning/teleop_imitation.html) · [DreamGen](https://arxiv.org/abs/2505.12705)
- [Jetson Thor](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-thor/) · [Orin NX](https://developer.nvidia.com/downloads/jetson-orin-nx-series-data-sheet)
- [Guía de desarrollo del G1](https://docs.westonrobot.com/tutorial/unitree/g1_dev_guide/) · [UnifoLM-VLA](https://github.com/unitreerobotics/unifolm-vla)
- [WBC-AGILE](https://github.com/nvidia-isaac/WBC-AGILE) · [arXiv:2603.20147](https://arxiv.org/abs/2603.20147)
