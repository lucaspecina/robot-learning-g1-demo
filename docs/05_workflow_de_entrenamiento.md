# El flujo de entrenamiento que vamos a seguir

> Decisión tomada el 10 de agosto de 2026: seguimos el **flujo punta a punta
> oficial de NVIDIA para GR00T + Unitree G1**, sin inventar un camino propio.
> Fuente: <https://docs.nvidia.com/learning/physical-ai/gr00t-e2e-workflow/latest/>

## Por qué este y no otro

Porque es de NVIDIA, está escrito **para el robot exacto que vamos a comprar**,
usa las mismas piezas que ya tenemos instaladas, y **el equipo socio también trabaja
dentro del ecosistema NVIDIA**. Seguir el mismo camino nos da vocabulario común
con ellos y evita fragmentar el stack.

La tarea que propone es deliberadamente simple: **el G1, parado en equilibrio
activo, agarra una manzana de una mesa y la deja en un plato.** No es la tarea
final del despliegue, y está bien que así sea: primero se recorre el molde con
algo medible, después se cambia la tarea.

---

## 1. AGILE y GR00T no compiten

Es la confusión más natural y conviene sacarla de encima primero. Y hay que
decirla con precisión, porque la versión corta —*"AGILE entrena las piernas"*—
**es incorrecta**.

### Qué entrena AGILE, según su propia tabla de tareas

| Tarea | Qué articulaciones controla |
|---|---|
| Locomoción | **solo piernas** (12) |
| Locomoción con altura | solo piernas (12) |
| Pararse desde el suelo | **todas** |
| Imitación de movimiento | **todas** |
| **Agarrar y colocar** | **solo cuerpo superior** |

**AGILE es el taller de entrenar por refuerzo, no el taller de las piernas.**
Entrena lo que se defina en la configuración de la tarea, y **también entrena
manipulación**: en esa última tarea, textual del paper, *"una política de
refuerzo controla solamente el brazo derecho y la cintura"*.

Lo que sí es específico de la tarea de locomoción es el patrón desacoplado:
*"la política de refuerzo controla exclusivamente las articulaciones de las
piernas, mientras las de cintura y brazos se aleatorizan durante el
entrenamiento"* — deliberadamente, para que en despliegue otro controlador
(cinemática inversa o un VLA) se haga cargo del torso y los brazos.

### La diferencia real: cómo aprende cada uno

| | **AGILE** | **Flujo de GR00T** |
|---|---|---|
| Cómo aprende | **probando**, guiado por una recompensa | **copiando** demostraciones |
| Qué datos pide | ninguno: sólo simulación | demostraciones grabadas |
| Qué recibe el modelo | el estado del cuerpo | imagen + instrucción en texto |
| Para nosotros | la policy de locomoción **ya está hecha, se consume** | **es lo que vamos a entrenar** |

### Y se apilan

En el flujo de GR00T hay un paso llamado *"teleoperación y control de cuerpo
completo"*: ese control de cuerpo completo **es la policy de locomoción de
AGILE, corriendo congelada por debajo**. Mientras el operador mueve los brazos,
las piernas se sostienen solas y compensan el peso desplazado.

### Hay un tercer camino que conviene no perder de vista

La tarea 5 de AGILE **es un camino de entrenamiento de manipulación completo y
sin teleoperar**: se entrena por refuerzo un experto que controla brazo y
cintura usando información privilegiada, ese experto **genera cien
demostraciones solo**, y con eso se afina el VLA.

O sea que para entrenar manipulación hay dos rutas, y las dos son de NVIDIA:
**teleoperar y copiar**, o **entrenar un experto y destilarlo**. La segunda no
necesita casco ni operador humano.

Detalle del taller de AGILE en
[`../g1/AGILE_TRAINING_REFERENCE.md`](../g1/AGILE_TRAINING_REFERENCE.md).

---

## 2. El flujo, paso a paso

Son **dos recorridos que se espejan**, de tres a seis horas cada uno.

### 2.1 En simulación — se puede hacer sin el robot

| Paso | Qué se hace | Qué produce |
|---|---|---|
| **1. Preparar Isaac Lab Arena** | instalar el marco donde ya vienen el G1, la mesa, la manzana y el plato | entorno andando |
| **2. Revisar la escena** | entender cómo se define la tarea: qué cuenta como éxito, qué se aleatoriza entre intentos | **saber diseñar una tarea** |
| **3. Teleoperar** | hacer la tarea a mano —con casco o con teclado— varias veces, mientras AGILE sostiene las piernas | episodios grabados |
| **4. Exportar** | convertir del formato interno al que consume el modelo | carpeta con imágenes, articulaciones y acciones |
| **5. Afinar** | mostrarle las demostraciones a GR00T ya entrenado | un modelo que sabe **esa** tarea |
| **6. Evaluar** | ponerlo a hacerla solo, muchas veces, desde posiciones distintas | **la tasa de éxito: el resultado real** |

El paso 2 parece de relleno y no lo es: es donde se aprende a **definir** una
tarea, que es lo que hará falta para inventar una del sitio real.

### 2.2 Con el robot real

Mismo esqueleto, con tres diferencias:

- se agrega **preparar la Jetson Thor**, la computadora que va sobre el robot;
- se agrega una etapa entera de **seguridad**, que en simulación no existe
  porque nada se rompe;
- al final, además de evaluar, hay que **convertir el modelo al formato de
  despliegue y cargarlo en el robot**.

El corazón es idéntico: **teleoperar → grabar → exportar → afinar.**

---

## 3. Dónde encaja cada cosa que ya investigamos

Las piezas sueltas que fuimos encontrando son estaciones de este mismo camino:

| Lo que investigamos | Dónde entra |
|---|---|
| Quest 3 y CloudXR | paso 3, teleoperar |
| Conversión de HDF5 a formato LeRobot | paso 4, exportar |
| Afinar por menos de 20 USD en placa alquilada | paso 5 |
| Isaac Lab Mimic: 10 demostraciones → cientos | entre el 3 y el 5, para escalar |
| Los 5 cuadros por segundo sobre la Thor | despliegue, en el flujo real |
| Cosmos Reason como verificador visual | evaluación, y como crítico al generar datos |

---

## 4. Lo que nos falta para empezar, y la decisión de hardware

Una sola cosa: **una placa de video que aguante el paso de teleoperación** —si
elegimos la ruta de teleoperar. La ruta del experto por refuerzo no la necesita.

CloudXR exige generación Ada o Blackwell. Caminos posibles: una máquina física
local, AWS con L40S, o Azure con RTX PRO 6000 Blackwell en vista previa; los dos
de nube quedan lejos de Argentina y arrastran latencia. Detalle en
[`04_generacion_de_datos.md`](04_generacion_de_datos.md).

**Con el robot real ese problema desaparece**, porque teleoperar hardware no
exige renderizar un mundo 3D: se transmite el video de las cámaras y todo ocurre
en la misma red local.

### Dos necesidades de hardware que no hay que confundir

| | **La máquina donde corre Isaac** | **La máquina que dibuja para el casco** |
|---|---|---|
| Para qué | desarrollar navegación, percepción, el agente | teleoperar contra la simulación |
| Qué la limita hoy | procesador, memoria, y que la T4 deforma el barrido del LiDAR | generación de la placa |
| Qué la resuelve | una A10 (Ampere) alcanza | **RTX PRO 6000 o RTX 5090 o superior; la A10 no sirve** |

> **Verificado el 13-ago-2026** contra la
> [documentación de CloudXR en Isaac Lab](https://isaac-sim.github.io/IsaacLab/main/source/how-to/cloudxr_teleoperation.html),
> que pide textualmente *"1x RTX PRO 6000 (o equivalente ej. 1x RTX 5090) o
> superior"*, más CPU de 16 núcleos clase Threadripper Pro y 64 GB de RAM, y
> sostener 45 FPS con física a 120 Hz. **Una RTX 5080 simula bien pero no llega
> a este requisito.**

**Y el detalle que ordena la decisión: en el flujo de NVIDIA esas dos máquinas
son la misma**, porque se teleopera contra la simulación. Así que subir a A10
mejora el trabajo de navegación pero **no habilita el entrenamiento**, y haría
falta otra máquina después.

No se puede decidir el hardware sin decidir antes qué se hace primero.

---

## 5. Cómo lo vamos a recorrer

1. **Hacer el flujo de simulación tal cual, con la manzana y el plato.** Sin
   cambiar la tarea, sin usar el sitio real, sin optimizar nada. Aunque el
   resultado sea mediocre.
2. **Anotar cuánto cuesta cada paso** en tiempo real, no en teoría: cuántos
   minutos por demostración, cuántas salen mal, cuánto tarda el ajuste.
3. **Recién entonces**, aplicar el mismo molde a una tarea del sitio real.

Es la misma progresión que funcionó con navegación: primero entender el
mecanismo en la salita de cuatro paredes, después discutir de igual a igual.

## Fuentes

- [Flujo punta a punta GR00T + G1](https://docs.nvidia.com/learning/physical-ai/gr00t-e2e-workflow/latest/)
- [Isaac Lab Arena](https://github.com/isaac-sim/IsaacLab-Arena)
- [Isaac GR00T](https://github.com/NVIDIA/Isaac-GR00T)
- [AGILE, arXiv:2603.20147](https://arxiv.org/abs/2603.20147)
