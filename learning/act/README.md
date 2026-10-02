# ACT

ACT (Action Chunking with Transformers) mira la cámara y la posición de los
brazos, y predice de una vez un bloque de acciones futuras en vez de una sola.
Predecir en bloque suaviza el movimiento y tolera una inferencia lenta: en el
Jetson del robot, sin GPU, un paso tarda unos 312 ms, y alcanza porque cada
predicción cubre 48 acciones.

## Lo que ya existe

La primera política (`act_cheap_v1`) se entrenó fuera de este repositorio,
sobre episodios reales de teleoperación con una sola cámara de cabeza.
Configuración relevante:

| Parámetro | Valor |
|---|---|
| Acciones por bloque (`chunk_size`) | 48 |
| Promedio temporal entre bloques | desactivado |
| Codificador de imagen | ResNet18 preentrenada en ImageNet |
| Dimensión del modelo | 192 |
| Latente del estilo (VAE) | 32, peso KL 10 |

Se evaluó offline sobre episodios separados del entrenamiento, con el
notebook que hoy vive en el robot.

## Con qué datos se entrenó

Reconstruido el 2-oct-2026 a partir de las estadísticas que el modelo guarda:

- Tarea `pick_mameluco`, 65 episodios grabados (`episode_0008` a
  `episode_0072`), **sin** `0008`, `0015`, `0028` y `0058`. Quedan 61
  episodios y 22.251 cuadros, los mismos que dice el modelo.
- Con esa selección, la media y el desvío de estado y acción coinciden con los
  del modelo con diferencias menores a 1e-7.
- La cámara no se compara: LeRobot la normaliza con las estadísticas de
  ImageNet (`use_imagenet_stats`), no con las del dataset.
- Entrenamiento: 54 episodios. Evaluación: los 7 restantes (índices 0, 10,
  ..., 60).

Todo esto está en [`split.json`](split.json). Para regenerar el dataset:

```bash
python -m toolkit.dataset.convert_unitree \
    --raw-dir ~/Desktop/episodios/pick_mameluco \
    --repo-id local/pick_mameluco_act_v1 \
    --skip-episodes episode_0008 episode_0015 episode_0028 episode_0058
``` Una lección de esa evaluación: la política
**no normaliza internamente**; sin su preprocesador y postprocesador de
LeRobot, los números salen inválidos.

## Próximo paso

Reproducir este entrenamiento desde el repo, con un comando por etapa, y
comprobar que la política reentrenada da los mismos números de evaluación.
Ese es el criterio para dar por cerrado el flujo.
