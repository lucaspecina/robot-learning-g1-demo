# Ideas: darle memoria a ACT

> Ideas de Lucas, octubre de 2026. **Las citas todavía no están verificadas**
> contra los papers; se marcan como verificadas a medida que se chequean.
> Objetivo de fondo: mejorar la ACT actual y llegar a una arquitectura propia.

## El punto de partida

ACT mira **sólo el instante actual**: las imágenes y los ángulos de ahora. No
sabe qué hizo hace un segundo. En el paper original son 4 cámaras; en nuestro
modelo es una sola, que se vuelve 240 tokens por instante (L3).

Se eligió así a propósito, para evitar la **confusión causal**: si la red ve su
propio pasado, puede aprender el atajo "seguí haciendo lo que venías haciendo"
en vez de mirar la escena (de Haan et al. 2019, *Causal Confusion in Imitation
Learning*; semana 3 del curso de ETH).

**El costo:** no puede resolver tareas donde la escena se ve igual pero el
momento es distinto ("¿ya hice una palada o dos?").

## Seis formas de darle memoria

### 1. Historia como observación

Meter los tokens de los últimos N instantes en la misma secuencia, con atención
que deja a cada instante mirar los anteriores.

- **Cuidado con el costo:** con imágenes, cada instante agrega muchos tokens
  (240 en nuestro modelo; unos 1200 con 4 cámaras), así que la ventana tiene
  que ser corta.
- **Ya se hace:** Octo (atención causal por bloques de tiempo), RT-1 (6
  cuadros), PRISM 2026 (resume la historia larga en niveles).

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
- Versiones modernas: Mamba Policy (modelos de espacio de estados) y ReCAT 2026,
  que mezcla capas recurrentes con una capa de atención (66,7% contra 8,3% del
  mejor método con historia corta, en robot real).

### 5. Un token de memoria que el transformer se pasa a sí mismo

Transformer adentro, recurrencia afuera. Viene de los LLM (Recurrent Memory
Transformer).

- En robots: ReMem-VLA y μVLA propagan tokens de memoria latentes; DRAM 2026 es
  un módulo de memoria recurrente que se enchufa a distintas políticas.
- Costo: hay que entrenar recorriendo instantes seguidos, no cuadros sueltos.

### 6. Mantener la decisión entre bloques

La idea original: reusar la `z` del bloque anterior para que el robot no cambie
de modo a mitad de camino.

- Con nuestra ACT no anda: `z` vale cero al predecir y el modelo la ignora
  (*posterior collapse*, notebook L3).
- Pero el problema es real. Bidirectional Decoding (Finn et al., ICLR 2025)
  sortea varios bloques candidatos y elige el coherente con lo que venía
  haciendo. HULC (Mees et al. 2022) mantiene un plan latente durante un tramo,
  más cerca de la idea original.

## Lo que atraviesa todo

**Memoria sin atajo y sin que explote el costo.** HALO 2026 lo resume en dos
problemas: las correlaciones espurias con el pasado y los errores que se
acumulan en la memoria. Su respuesta es atención dispersa, que sólo recupera lo
relevante (hasta 8 minutos de historia).

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
