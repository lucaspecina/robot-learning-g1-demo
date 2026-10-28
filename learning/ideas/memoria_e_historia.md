# Ideas: darle memoria a ACT

> Ideas de Lucas, octubre de 2026. Objetivo de fondo: mejorar la ACT actual y
> llegar a una arquitectura propia. Las 15 referencias se verificaron contra
> los papers el 27-oct-2026 (tabla al final); los matices quedaron marcados
> en el texto. De los papers de 2026 se leyó sólo el abstract.

## El punto de partida

ACT mira **sólo el instante actual**: las imágenes y los ángulos de ahora. No
sabe qué hizo hace un segundo. En el paper original son 4 cámaras; en nuestro
modelo es una sola, que se vuelve 240 tokens por instante (L3).

Se eligió así a propósito, para evitar la **confusión causal**: si la red ve su
propio pasado, puede aprender el atajo "seguí haciendo lo que venías haciendo"
en vez de mirar la escena (de Haan et al. 2019, *Causal Confusion in Imitation
Learning*; semana 3 del curso de ETH). En ese paper el atajo es sobre todo la
**acción anterior**: con historia, la política rinde mejor sobre datos
grabados y mucho peor al usarla de verdad.

El paper de ACT lo justifica en una sola frase (el chunking modela
comportamientos que dependen del pasado "sin introducir la confusión causal"
de las políticas con historia, citando a de Haan), pero no tiene un
experimento que compare con y sin historia.

**El costo:** no puede resolver tareas donde la escena se ve igual pero el
momento es distinto ("¿ya hice una palada o dos?").

## Seis formas de darle memoria

### 1. Historia como observación

Meter los tokens de los últimos N instantes en la misma secuencia, con atención
que deja a cada instante mirar los anteriores.

- **Cuidado con el costo:** con imágenes, cada instante agrega muchos tokens
  (240 en nuestro modelo; unos 1200 con 4 cámaras), así que la ventana tiene
  que ser corta.
- **Ya se hace:** Octo (atención causal por bloques de tiempo; usa sólo 2
  cuadros, porque más allá del primero adicional la ganancia caía mucho),
  RT-1 (6 cuadros), PRISM 2026 (resume la historia larga en niveles, hasta
  unos dos minutos, y usa atención con compuertas para reducir las
  correlaciones espurias con el pasado).

### 2. Imágenes recientes con detalle, viejas comprimidas

Las imágenes viejas pesan menos tokens que las recientes, o aparecen cada
tantos pasos.

- RT-1 ya comprime cada imagen a 8 tokens para poder ver historia.
- VPWEM 2026 hace casi exactamente esta idea: memoria de trabajo con los tokens
  recientes, y un transformer que condensa lo que sale de la ventana en unos
  pocos tokens de memoria.

### 3. Historia densa de ángulos

Un token por instante: barata. Pero es la historia más peligrosa, porque los
ángulos pasados son justo lo que permite el atajo de copiar. **Barato no quiere
decir seguro.**

### 4. Un estado latente tipo RNN

Un vector que resume todo el pasado, con costo fijo por paso.

- BC-RNN ya funcionaba mejor que redes sin memoria en robomimic (2021).
- Problemas: cuello de botella fijo, entrenamiento secuencial, y el atajo sigue
  ahí.
- Versiones modernas: ReCAT 2026, que mezcla capas recurrentes (Mamba-2) con
  una capa de atención causal: 66,7% contra 8,3% del mejor método con historia
  corta, en robot real (es su mejor variante, en 3 tareas de recordar, contar
  y medir tiempo).
- **Ojo con Mamba Policy:** usa modelos de espacio de estados para que Diffusion
  Policy sea más eficiente (80% menos parámetros), no como memoria del
  episodio: mira sólo 2 observaciones.
- En robomimic, BC-RNN le gana a BC en casi todo (Transport: 71,3% contra
  17,3%), con alguna excepción (Tool Hang con estado de baja dimensión).

### 5. Un token de memoria que el transformer se pasa a sí mismo

Transformer adentro, recurrencia afuera. Viene de los LLM (Recurrent Memory
Transformer).

- En robots: ReMem-VLA y μVLA propagan tokens de memoria latentes (μVLA lo
  firman dos autores del Recurrent Memory Transformer); DRAM 2026 es un módulo
  de memoria recurrente que se enchufa a distintas políticas sin reentrenar la
  base, aunque igual pide entrenar el módulo y la cabeza de acciones para cada
  tarea.
- Costo: hay que entrenar recorriendo instantes seguidos, no cuadros sueltos.

### 6. Mantener la decisión entre bloques

La idea original: reusar la `z` del bloque anterior para que el robot no cambie
de modo a mitad de camino.

- Con nuestra ACT no anda: `z` vale cero al predecir y el modelo la ignora
  (*posterior collapse*, notebook L3).
- Pero el problema es real. Bidirectional Decoding (Liu, ..., Finn, ICLR 2025)
  sortea varios bloques candidatos y elige con dos criterios: coherencia con lo
  que venía haciendo y contraste hacia adelante (que el plan futuro sea
  probable).
- HULC (Mees et al. 2022) sigue un plan latente global, más cerca de la idea
  original. El paper no dice cuánto dura; en su código oficial se replanifica
  cada 30 pasos (un segundo).

## Lo que atraviesa todo

**Memoria sin atajo y sin que explote el costo.** HALO 2026 lo resume en dos
problemas: las correlaciones espurias con el pasado y los errores que se
acumulan en la memoria. Su respuesta es atención dispersa, que sólo recupera lo
relevante (hasta 8 minutos de historia), más un objetivo auxiliar de
preguntas sobre el video, destilado de un modelo de visión y lenguaje, contra
las correlaciones espurias.

## Antes de probar cualquiera: cómo se va a medir

Notas para que una mejora se pueda demostrar, no sólo afirmar:

1. **Una referencia reproducible.** La ACT actual reentrenada desde el repo,
   con los mismos números de evaluación. Sin eso, una diferencia puede venir
   del entrenamiento y no de la arquitectura.
2. **Una tarea donde la memoria importe.** En `pick_mameluco` la escena
   probablemente alcanza para saber qué hacer en cada momento; ahí una política
   con memoria no tendría cómo mostrar ventaja. Hace falta al menos una tarea
   donde la escena se vea igual en dos momentos distintos (contar, repetir,
   esconder algo y buscarlo).
3. **Medir el atajo.** Además del éxito, probar si la política con historia
   copia su pasado: por ejemplo, perturbar la escena a mitad del episodio y ver
   si reacciona o sigue de largo.
4. **Lazo cerrado.** La evaluación offline (*teacher forcing*) le da a la red
   siempre el pasado verdadero, que es exactamente lo que esconde el problema
   de la confusión causal. La memoria se evalúa en simulación o en el robot.

## Referencias verificadas

| Idea | Paper | Link |
|---|---|---|
| Confusión causal | de Haan, Jayaraman, Levine. *Causal Confusion in Imitation Learning* (NeurIPS 2019) | [1905.11979](https://arxiv.org/abs/1905.11979) |
| ACT | Zhao, Kumar, Levine, Finn. *Learning Fine-Grained Bimanual Manipulation with Low-Cost Hardware* (2023) | [2304.13705](https://arxiv.org/abs/2304.13705) |
| 1 | Octo Model Team. *Octo: An Open-Source Generalist Robot Policy* (2024) | [2405.12213](https://arxiv.org/abs/2405.12213) |
| 1, 2 | Brohan et al. *RT-1: Robotics Transformer for Real-World Control at Scale* (2022) | [2212.06817](https://arxiv.org/abs/2212.06817) |
| 1 | Shah et al. *Scaling Short-Term Memory of Visuomotor Policies for Long-Horizon Tasks* (PRISM, 2026) | [2606.16178](https://arxiv.org/abs/2606.16178) |
| 2 | Lei et al. *VPWEM: Non-Markovian Visuomotor Policy with Working and Episodic Memory* (2026, RA-L) | [2603.04910](https://arxiv.org/abs/2603.04910) |
| 4 | Mandlekar et al. *What Matters in Learning from Offline Human Demonstrations for Robot Manipulation* (robomimic, 2021) | [2108.03298](https://arxiv.org/abs/2108.03298) |
| 4 | Cao et al. *Mamba Policy: Towards Efficient 3D Diffusion Policy with Hybrid Selective State Models* (2024) | [2409.07163](https://arxiv.org/abs/2409.07163) |
| 4 | Vanjani et al. *ReCAT: Remember, Count, and Time: Structured Recurrent Memory for Robot Manipulation* (2026) | [2609.35200](https://arxiv.org/abs/2609.35200) |
| 5 | Bulatov, Kuratov, Burtsev. *Recurrent Memory Transformer* (NeurIPS 2022) | [2207.06881](https://arxiv.org/abs/2207.06881) |
| 5 | Li et al. *ReMem-VLA: Empowering Vision-Language-Action Model with Memory via Dual-Level Recurrent Queries* (2026) | [2603.12942](https://arxiv.org/abs/2603.12942) |
| 5 | Cherepanov et al. *μVLA: On Recurrent Memory for Partially Observable Manipulation in VLA Models* (2026) | [2606.12497](https://arxiv.org/abs/2606.12497) |
| 5 | Zhao et al. *DRAM: Delta-rule Recurrent Associative Memory for Robot Manipulation Policies* (2026) | [2609.32453](https://arxiv.org/abs/2609.32453) |
| 6 | Liu, ..., Finn. *Bidirectional Decoding: Improving Action Chunking via Guided Test-Time Sampling* (ICLR 2025) | [2408.17355](https://arxiv.org/abs/2408.17355) |
| 6 | Mees, Hermann, Burgard. *What Matters in Language Conditioned Robotic Imitation Learning over Unstructured Data* (HULC, RA-L 2022) | [2204.06252](https://arxiv.org/abs/2204.06252) |
| Transversal | Shah et al. *Memory Retrieval in Visuomotor Policies for Long-Horizon Robot Control* (HALO, 2026) | [2606.25136](https://arxiv.org/abs/2606.25136) |

No verificado: en qué congreso salieron los papers de 2026 (salvo VPWEM, en
RA-L), y los números de esos papers más allá de su abstract.
