# Cómo se generan los datos para entrenar manipulación

> Investigación de agosto de 2026. Continúa
> [`03_arquitectura_inteligencia_humanoide.md`](03_arquitectura_inteligencia_humanoide.md),
> que estableció que la única capa que vamos a entrenar es la manipulación.
> Este documento responde de dónde salen los datos para eso, y qué hardware
> hace falta para cada camino.

## TL;DR

1. **Hay tres formas de generar demostraciones**: teleoperar a mano, entrenar un
   experto por refuerzo que las genere solo, o generarlas sintéticamente con
   modelos de video. Las tres están documentadas y las tres se pueden combinar.
2. **Con diez demostraciones alcanza para empezar.** Isaac Lab Mimic las
   multiplica a cientos. Ese número —diez— es el que hay que retener.
3. **El casco no se puede usar en Azure. Con ninguna VM.** CloudXR exige placas
   de generación Ada o Blackwell, y la familia de visualización de Azure llega
   hasta Ampere. Es un bloqueo de hardware, no de configuración.
4. **Pero hay un experimento completo que sí corre hoy en la T4**, sin casco y
   sin cámaras, y enseña el ciclo entero.
5. **Cosmos Reason no es un planificador de misiones.** Es un verificador visual.
   Usarlo para planificar en texto sería forzarlo.

---

## 1. Los tres caminos para conseguir demostraciones

### 1.1 Teleoperar a mano

Una persona mueve el robot simulado y el sistema graba todo: imágenes,
articulaciones y acciones. Sale movimiento humano y natural, pero cada episodio
cuesta tiempo de persona.

Isaac Lab lo soporta con **muchos dispositivos**, y sólo uno de ellos necesita
placa cara:

| Dispositivo | ¿Necesita placa Ada o superior? |
|---|---|
| Teclado | no |
| Mouse 3D (SpaceMouse) | no — **la documentación lo recomienda** porque da demostraciones más suaves que el teclado |
| Mando tipo consola | no |
| Haply, con retorno de fuerza | no |
| Guantes Manus + trackers | no |
| **Casco de realidad virtual** | **sí** |

### 1.2 Un experto artificial que las genera solo

Es el camino del paper de AGILE, su tarea 5. Se entrena por refuerzo una
política que ya sabe hacer la tarea porque **hace trampa** —conoce la posición
exacta del objeto—, y después esa política **genera 100 trayectorias exitosas
sola**, con aleatorización de física y de apariencia, grabando imagen,
propiocepción y acciones. **Sin ninguna teleoperación humana.**

Después se afina el modelo de manipulación reemplazando la información
privilegiada por imagen más instrucción en texto.

Ventaja: costo humano casi cero una vez entrenado el experto. Costo: hay que
saber escribir la recompensa, y el experto puede aprender a explotar errores del
simulador.

### 1.3 Generación sintética con modelos de video

**GR00T-Dreams**: se generan trayectorias con un modelo de video a partir de
imágenes, y **Cosmos Reason actúa de crítico** descartando las malas. Cero
teleoperación y cero refuerzo, pero calidad más incierta. Pide placas grandes.

### 1.4 Cuál conviene

| | Teleoperar | Experto por refuerzo |
|---|---|---|
| Cuándo | tareas de destreza o mucho contacto, difíciles de escribir como recompensa | tareas especificables: agarrar y colocar, alcanzar, empujar |
| Costo humano | alto | casi cero después de entrenar |
| Riesgo principal | consistencia de las demostraciones | diseñar la recompensa |
| Hardware de entrada | casco, mouse 3D o guantes | ninguno |

---

## 2. Multiplicar pocas demostraciones: el número mágico es diez

**Isaac Lab Mimic** parte cada demostración en segmentos centrados en el objeto
y los adapta a escenas nuevas. La documentación es explícita: **hacen falta unas
diez demostraciones exitosas**.

El flujo son tres comandos: grabar, anotar los límites de cada subtarea —hay
modo automático—, y generar. De diez salen cientos.

**SkillGen**, nuevo en la versión 2.3, es la alternativa más capaz: combina los
segmentos humanos con planificación de movimiento acelerada por placa, y produce
demostraciones **libres de colisión**. Pero exige anotar a mano el inicio *y* el
fin de cada subtarea, y pide **24 GB de memoria de video como mínimo**.

Tasas esperables que publica la documentación: **40 a 70 % de éxito al generar**,
y **40 a 85 % en la política resultante**.

Y hay una pieza específica para nuestro caso: un guión que toma un conjunto de
datos de **manipulación** y lo convierte en escenarios de **agarrar, navegar y
colocar**, aleatorizando lugares y obstáculos. Es la forma de pasar de
manipulación a loco-manipulación **sin volver a teleoperar**.

---

## 3. El camino del casco, y por qué Azure no sirve

### 3.1 La buena noticia

**El Quest 3 está soportado**, y el flujo oficial de NVIDIA para el G1 lo
recomienda **por encima del Vision Pro**. Además, **los adaptadores de mandos de
Quest para el G1 ya están en la versión 2.3.2 que tenemos instalada** — llegaron
en noviembre de 2025 y traen control de locomoción y altura con los joysticks, y
de los dedos con los gatillos.

En el Quest **no hay que instalar nada**: se abre un cliente web en el navegador
del casco. La conexión es por wifi local, con router moderno.

### 3.2 La mala

Todo eso pasa por **CloudXR**, un servicio que corre en la máquina del
simulador, renderiza y transmite al casco. Y sus requisitos son duros:

| Fuente | Requisito |
|---|---|
| Isaac Lab, guía de CloudXR | una RTX PRO 6000 o equivalente, 16 núcleos, 64 GB de RAM |
| Isaac Teleop | RTX 6000 Blackwell, o dos RTX 6000 Ada |
| CloudXR | **generación Ada o Blackwell**, con soporte de codificación de video. Textual: *las placas de sólo cómputo, como A100 y H100, no están soportadas* |
| Isaac Sim 5.1 | mínimo RTX 4080 y **32 GB de RAM de sistema** |

**Nuestra T4 es Turing, dos generaciones por debajo del mínimo.**

Y el dato que cierra la puerta: **la familia de visualización de Azure llega
hasta A10, que es Ampere.** No hay ninguna VM pública de Azure con placa Ada o
Blackwell. Las familias de cómputo con A100 o H100 tampoco sirven, porque Isaac
Sim las excluye explícitamente por no tener núcleos de trazado de rayos.

**Conclusión: el camino del casco no se desbloquea cambiando de VM. Se
desbloquea con una máquina física con placa RTX moderna.**

### 3.3 Un atajo útil, con letra chica

Existe **IWER**, un emulador que hace que el navegador de escritorio se haga
pasar por un Quest 3, con mouse y teclado. Permite desarrollar y depurar todo el
canal de teleoperación **sin tener el casco puesto**.

Pero emula el **cliente**, no el servidor: **sigue haciendo falta CloudXR**, y
por lo tanto la placa. Sirve para no depender del casco, no para esquivar el
hardware.

---

## 4. Qué sí se puede hacer hoy con lo que tenemos

Un experimento completo que **no toca el casco, ni las cámaras, ni el G1**, y que
valida el ciclo entero de generación de datos:

1. Una tarea de manipulación con **brazo simple** —apilar cubos— en vez del
   humanoide.
2. Grabar **diez demostraciones con el teclado**, sin ventana gráfica y sin
   cámaras.
3. Anotar los límites de subtarea en modo automático.
4. **Generar cien** con Mimic.
5. Entrenar una política por imitación y medir cuántas veces le sale.

Eso enseña el ciclo *demostración → multiplicación → política* de punta a punta.
Recién después tiene sentido escalar al G1 y a las cámaras.

### 4.1 Qué falla primero, en orden

1. **La memoria del sistema.** Tenemos 28 GB; el mínimo declarado de Isaac Sim
   es **32 GB**. Ya estamos por debajo antes de empezar — y eso explica que el
   simulador muriera por falta de memoria el 10 de agosto.
2. **El procesador.** Cuatro núcleos es el mínimo absoluto, y ya está medido que
   es el cuello de botella.
3. **Encender las cámaras.** En cuanto haya que grabar imagen —que es
   obligatorio para el conjunto de datos final— la T4 se desploma.
4. **La memoria de video.** 16 GB. SkillGen pide 24 para un solo entorno.
5. **Afinar el modelo.** Pide **48 GB de memoria de video y 128 GB de RAM**.

---

## 5. Del dato crudo al modelo entrenado

El recorrido completo, para tenerlo claro:

1. **Grabar** produce un archivo HDF5.
2. **Convertir** a formato LeRobot, que es lo que consumen los tres modelos
   grandes de manipulación. Queda una carpeta con tablas de estados y acciones,
   videos de las cámaras y metadatos.
3. **Afinar** el modelo. Unas 2 a 3 horas para 20.000 pasos.

Dos detalles que arruinan el trabajo si se ignoran:

- Cada episodio **necesita video de una cámara en la cabeza del robot**. Sin eso
  el conjunto de datos no es válido.
- El **horizonte de acción queda horneado en el modelo** y tiene que coincidir
  entre entrenamiento y ejecución.

La documentación recomienda **correr la conversión sobre una grabación corta
antes de recolectar en masa**. Es el consejo más barato del documento: evita
descubrir que doscientas demostraciones son inservibles.

Para una política decente sobre el G1, la referencia oficial recomienda **unas
400 demostraciones de calidad**.

---

## 6. Cosmos Reason: qué es y qué no

**Qué es**: un modelo que mira video o imágenes y razona sobre espacio, tiempo y
física de sentido común. Viene en 2, 8 y 32 mil millones de parámetros, con
**licencia que permite uso comercial**.

**Está confirmado que es el motor de comprensión dentro de GR00T N1.7** — la
parte que interpreta la escena antes de generar el movimiento.

**Los tres usos que NVIDIA declara** son análisis de video, curación y anotación
de datos, y razonamiento dentro de un modelo de acción.

**Qué no es**: un planificador de misiones en texto. **No hay ningún ejemplo
oficial** del patrón "orden en lenguaje natural más catálogo de capacidades,
devolveme una lista de pasos". Todos los ejemplos documentados tienen imagen o
video de entrada.

Donde sí encaja en nuestra arquitectura es como **verificador visual de la
ejecución**: mirar el video del robot y responder si el paso anterior salió bien,
si el objeto está donde debería, si la acción siguiente es físicamente posible.
Ese uso está documentado —lo usan como crítico para filtrar trayectorias
sintéticas— y es exactamente la capacidad que hoy nos falta.

**Cómo usarlo desde la T4**: no corre localmente —pide 24 GB y arquitecturas más
nuevas— pero está disponible como **API hospedada con créditos gratuitos y uso
comercial permitido**. Fricción cero.

**Aviso de vigencia**: los repositorios de Cosmos Reason 1 y 2 ya avisan que no
están en desarrollo activo y redirigen a **Cosmos 3**, publicado en mayo de 2026,
que unifica razonamiento y generación. Conviene mirar hacia allá antes de
construir sobre Reason 2.

---

## 7. Qué implica para el hardware

Resumiendo las tres necesidades, que son distintas:

**Para simular y desarrollar el stack**: la VM de Azure sirve, pero conviene
subir a A10 —que es Ampere y destraba además la compensación de movimiento del
LiDAR— y sobre todo salir de los 28 GB de RAM.

**Para teleoperar con el casco y generar datos con cámaras**: hace falta una
**máquina física con placa RTX Ada o Blackwell**. Azure no lo ofrece.

**Para afinar el modelo**: 48 GB de memoria de video. Se alquila por horas
cuando haga falta; no se necesita tener prendido.

Son tres máquinas distintas, o una muy grande. Conviene decidirlo con esa
separación en la cabeza.

## Fuentes

- [CloudXR en Isaac Lab](https://isaac-sim.github.io/IsaacLab/main/source/how-to/cloudxr_teleoperation.html) · [requisitos de CloudXR](https://docs.nvidia.com/cloudxr-sdk/latest/requirement/runtime_req.html) · [Isaac Teleop](https://nvidia.github.io/IsaacTeleop/main/getting_started/quick_start.html)
- [Teleoperación e imitación en Isaac Lab](https://isaac-sim.github.io/IsaacLab/main/source/overview/imitation-learning/teleop_imitation.html) · [SkillGen](https://isaac-sim.github.io/IsaacLab/main/source/overview/imitation-learning/skillgen.html) · [dispositivos](https://isaac-sim.github.io/IsaacLab/main/source/api/lab/isaaclab.devices.html)
- [Adaptadores de Quest para el G1 (PR #3950)](https://github.com/isaac-sim/IsaacLab/pull/3950) · [Isaac Lab 2.3.2](https://github.com/isaac-sim/IsaacLab/releases/tag/v2.3.2)
- [Flujo GR00T + G1: teleoperación](https://docs.nvidia.com/learning/physical-ai/gr00t-e2e-workflow/latest/simulation-workflow/sim-teleop-and-wbc.html) · [exportar datos](https://docs.nvidia.com/learning/physical-ai/gr00t-e2e-workflow/latest/simulation-workflow/sim-data-export.html) · [afinar](https://docs.nvidia.com/learning/physical-ai/gr00t-e2e-workflow/latest/simulation-workflow/groot-fine-tuning-sim.html)
- [Isaac Lab Arena](https://github.com/isaac-sim/IsaacLab-Arena) · [requisitos de Isaac Sim 5.1](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/requirements.html)
- [Cosmos Reason 2](https://github.com/nvidia-cosmos/cosmos-reason2) · [modelo 2B](https://huggingface.co/nvidia/Cosmos-Reason2-2B) · [API hospedada](https://build.nvidia.com/nvidia/cosmos-reason2-8b) · [Cosmos 3](https://github.com/nvidia/Cosmos) · [GR00T N1.7](https://huggingface.co/blog/nvidia/gr00t-n1-7)
- [AGILE, arXiv:2603.20147](https://arxiv.org/abs/2603.20147) · [familia NV de Azure](https://learn.microsoft.com/en-us/azure/virtual-machines/sizes/gpu-accelerated/nv-family)
