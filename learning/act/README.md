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
notebook que hoy vive en el robot. Una lección de esa evaluación: la política
**no normaliza internamente**; sin su preprocesador y postprocesador de
LeRobot, los números salen inválidos.

## Próximo paso

Reproducir este entrenamiento desde el repo, con un comando por etapa, y
comprobar que la política reentrenada da los mismos números de evaluación.
Ese es el criterio para dar por cerrado el flujo.
