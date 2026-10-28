# Mapa de situación: stack, flujos y estrategias abiertas

> **Este es el documento de entrada.** Los otros profundizan; éste dice dónde
> estamos parados y qué opciones hay sobre la mesa. Es un documento **vivo**:
> nada acá está cerrado salvo lo que diga explícitamente "decidido".
>
> Última actualización: 10 de agosto de 2026.

---

## 1. El stack, pieza por pieza

Quién hace qué, y en qué estado está cada decisión.

| Capa | Qué hace | Qué usamos | Estado |
|---|---|---|---|
| **Entender la orden** | recibe una orden en castellano y arma un plan de pasos | modelo de lenguaje grande, vía API | **decidido**, funcionando |
| **Decidir y validar** | valida el plan, ejecuta un paso por vez, mide resultados | **código propio** (`agent/agent.py`, `skill_catalog.py`) | **decidido**, funcionando |
| **Verificar visualmente** | mirar el video y decir si el paso salió bien | *falta*. Candidato: Cosmos Reason por API | **abierto** |
| **Percibir** | reconocer objetos, medir distancias | RT-DETR local + Grounding DINO remoto | funcionando |
| **Navegar** | de un punto a otro, esquivando | Nav2 (NavFn + DWB), SLAM Toolbox, AMCL | funcionando, con bloqueos medidos |
| **Manipular** | agarrar, sostener, soltar | *falta*. Candidato: GR00T afinado | **abierto — es lo único que vamos a entrenar** |
| **Caminar** | convertir velocidad en pasos sin caerse | policy *student* de WBC-AGILE | **decidido**, se consume sin entrenar |

**Decisión de ecosistema (10-ago-2026):** nos quedamos **dentro de NVIDIA**.
Isaac para simular, Cosmos para razonar y multiplicar datos, GR00T para
manipular, AGILE para entrenar por refuerzo. Motivo: el equipo socio trabaja ahí
adentro y fragmentar el stack nos deja sin vocabulario común.

---

## 2. Los flujos, y en qué se diferencian

Es donde más nos confundimos. Los dos son de NVIDIA, los dos usan Isaac, y los
dos terminan en el robot — pero **no hacen lo mismo**.

| | **AGILE** | **Flujo punta a punta de GR00T** |
|---|---|---|
| Qué es | un taller para **entrenar por refuerzo** | un curso guiado para **afinar un modelo copiando demostraciones** |
| Cómo aprende el modelo | probando millones de veces, guiado por una recompensa | imitando ejemplos grabados |
| Qué articulaciones toca | **las que definas**: solo piernas, todas, o solo brazos | brazos y manos |
| Qué datos pide | ninguno, sólo simulación | demostraciones grabadas |
| Etapas | preparar → entrenar → evaluar → desplegar | teleoperar → grabar → exportar → afinar → evaluar |

**Y se apilan.** En el paso de teleoperación del flujo de GR00T, la policy de
locomoción de AGILE corre **congelada por debajo** sosteniendo el cuerpo
mientras el operador mueve los brazos.

⚠️ **Error que ya cometimos:** decir *"AGILE entrena las piernas"*. Es falso.
AGILE entrena lo que se defina, y su tarea de agarrar y colocar controla **solo
el cuerpo superior**. Detalle en
[`../g1/AGILE_TRAINING_REFERENCE.md`](../g1/AGILE_TRAINING_REFERENCE.md).

---

## 3. Estrategias posibles para conseguir manipulación

**Ninguna está elegida todavía.** Las tres son caminos reales y documentados.

### A. Teleoperar en simulación

Te ponés el casco, hacés la tarea vos en el simulador, grabás. Diez
demostraciones alcanzan para empezar: Isaac Lab Mimic las multiplica a cientos.

- **A favor**: movimiento humano y natural; permite reiniciar y aleatorizar
  infinitamente; escala con las herramientas de multiplicación.
- **En contra**: **exige placa Ada o Blackwell** para renderizar al casco, y
  desde Argentina las opciones de nube arrastran 120 ms o más de latencia — con
  lag se graban demostraciones malas.
- **Qué haría falta**: una máquina física con RTX moderna, o aceptar la latencia.

### B. Experto por refuerzo que genera las demostraciones solo

Es la tarea 5 de AGILE. Se entrena una política que ya sabe hacer la tarea
porque conoce la posición exacta del objeto, y **esa política graba cien
demostraciones sin ninguna intervención humana**. Después se afina el modelo
reemplazando esa información privilegiada por imagen más texto.

- **A favor**: **no necesita casco, ni operador, ni placa para renderizar**.
  Costo humano casi nulo una vez entrenado el experto.
- **En contra**: hay que saber escribir la recompensa, y el experto puede
  aprender a explotar errores del simulador.
- **Qué haría falta**: placa para entrenar por refuerzo, no para renderizar.
  **Es la ruta que podría volver innecesaria toda la discusión del casco.**

### C. Teleoperar el robot real

Cuando llegue el G1. No hay mundo 3D que dibujar: se transmite el video de las
cámaras y las manos del operador mueven los brazos.

- **A favor**: **no necesita placa exótica** —corre en una PC normal— y todo
  ocurre en la misma red local, así que la latencia desaparece. Los datos son la
  verdad, sin brecha entre simulación y realidad.
- **En contra**: cada intento cuesta tiempo real, desgasta el hardware, y si el
  robot se cae, se cae. No se puede aleatorizar ni multiplicar.
- **Qué haría falta**: el robot, y la Jetson Thor.

**Lo más probable es que terminemos combinando B y C**: masa de datos en
simulación, y demostraciones reales para ajustar y validar. Es lo que hace el
flujo de NVIDIA.

---

## 4. Hardware: tres necesidades distintas que no hay que mezclar

| Para qué | Qué pide | Qué tenemos |
|---|---|---|
| **Correr Isaac y desarrollar el stack** | procesador, memoria, y Ampere o superior para el LiDAR | T4 de 4 núcleos y 28 GB — **por debajo del mínimo de Isaac Sim, que pide 32** |
| **Renderizar para el casco** | **RTX PRO 6000 o RTX 5090 o superior** | nada que califique |
| **Afinar el modelo** | 48 GB de memoria de video | nada — se alquila por horas, menos de 20 dólares |

⚠️ **Trampa que ya nos confundió:** en el flujo de NVIDIA se teleopera **contra
la simulación**, así que **la primera y la segunda son la misma máquina**. Subir
a A10 mejora navegación pero **no habilita el entrenamiento por teleoperación**.

Opciones que sí califican para el casco: RTX PRO 6000 Blackwell en Azure (vista
previa, sólo West US 2 y Sudeste Asiático), L40S en AWS (no hay en San Pablo), o
una máquina física local — **la única sin problema de latencia**.

---

## 5. Qué está decidido y qué está abierto

**Decidido**

- Nos quedamos en el ecosistema NVIDIA.
- Seguimos el flujo punta a punta oficial de GR00T + G1, sin inventar camino.
- No entrenamos locomoción: se consume la de AGILE.
- No preentrenamos ningún modelo: sólo afinamos.
- La arquitectura modular con catálogo cerrado se conserva — es la que Boston
  Dynamics y Google publicaron como patrón de referencia.

**Abierto**

- **Cuál de las tres estrategias de manipulación** (A, B o C).
- **Qué hardware comprar o alquilar**, que depende de la anterior.
- Si conviene subir la VM a A10 ahora — sirve para navegación, no para
  entrenamiento.
- Si Cosmos Reason entra como verificador visual.
- Qué manos va a traer el G1 — **condiciona todo lo de manipulación**.

**Para preguntarle al equipo socio**

- ¿Generan datos en simulación o sólo teleoperan el robot real?
- ¿Con qué máquina teleoperan?
- ¿Qué configuración de manos recomiendan comprar?
- ¿Por qué hay que reestampar el LiDAR y qué se pierde con eso?
- ¿Cómo piensan validar la navegación contra el gemelo del sitio real?

---

## 6. Errores que ya cometimos, para no repetirlos

Se dejan escritos a propósito: cada uno costó tiempo y volver a caer es fácil.

1. **"AGILE entrena las piernas"** — falso; entrena lo que se defina.
2. **"Azure no tiene ninguna placa que sirva para el casco"** — falso; tiene RTX
   PRO 6000 Blackwell en vista previa. Lo que no tiene es una cerca de acá.
3. **Mezclar "dónde corre el simulador" con "qué hace falta para el casco"** —
   son dos preguntas distintas, y la respuesta a una no responde la otra.
4. **Recomendar el razonador de Google** sin considerar que el equipo socio trabaja
   en el stack de NVIDIA.
5. **Suponer que el LiDAR sólo ve a su altura** — depende de la franja que se
   elija al aplanar la nube 3D; en nuestra simulación el `/scan` es un plano de
   verdad, pero eso es un artefacto del banco, no del robot real.

---

## Dónde seguir

- [`03_arquitectura_inteligencia_humanoide.md`](03_arquitectura_inteligencia_humanoide.md) — qué arquitectura entra en el cómputo del robot
- [`04_generacion_de_datos.md`](04_generacion_de_datos.md) — de dónde salen los datos y qué placa pide cada camino
- [`05_workflow_de_entrenamiento.md`](05_workflow_de_entrenamiento.md) — el flujo oficial que seguimos, paso a paso
- [`../g1/AGILE_TRAINING_REFERENCE.md`](../g1/AGILE_TRAINING_REFERENCE.md) — el taller de entrenamiento por refuerzo
- [`../g1/README.md`](../g1/README.md) — estado del sistema que ya funciona
