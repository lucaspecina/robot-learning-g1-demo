# notebooks/

Cada paso del flujo, para correrlo y mirarlo. La lógica vive en
[`toolkit/`](../toolkit/); los notebooks la llaman y grafican.

| Notebook | Qué se ve |
|---|---|
| `01_raw_recordings` | Qué trae una grabación de teleoperación: estado, acción, cámara, duraciones |
| `02_convert_to_lerobot` | La conversión a LeRobot, cómo pide los datos ACT y qué se pierde al comprimir |
| `03_which_data_trained_act` | Cómo se reconstruyó con qué episodios se entrenó la primera ACT |
| `04_validate_dataset` | Validación del dataset con las prácticas de los labs: integridad, señales, lag, éxito, deriva, visual, split y fuga |
| `05_offline_eval_act` | Evaluación offline de ACT contra referencias tontas, por horizonte y por articulación |

Se abren con el entorno del repo (`bash scripts/setup_learning.sh`):

```bash
source .venv/bin/activate
jupyter lab notebooks/
```

Los notebooks se versionan sin salidas: el filtro `nbstripout` (lo instala
`setup_learning.sh`) las borra al commitear y en tu copia quedan intactas. Es
importante porque las salidas incluyen imágenes del lugar real y el repo es
público.

## `learn/`: referencia para entender a fondo

Cada uno arranca con lo principal y termina con secciones "para profundizar".

| Notebook | Qué explica |
|---|---|
| `L3_1_act_data` | ACT · los datos: de la grabación al lote que entra al `forward` (ventanas, relleno, normalización) |
| `L3_2_act_vae_encoder` | ACT · el encoder del VAE: tokens, CLS, μ y σ, el sorteo de la `z`, el KL, y el colapso de nuestro modelo |
| `L3_3_act_decoder_policy` | ACT · el decoder (la policy): imagen → tokens, posiciones 2D, encoder, casilleros y mapas de cross-attention sobre la imagen |
| `L3_4_act_training` | ACT · el entrenamiento: un paso desarmado (error, gradientes) y un modelo desde cero con fotos de cómo aprende |
| `L3_5_act_inference` | ACT · en uso: `z = 0`, latencia, bloques contra *temporal ensembling*, y las perillas |
| `L1_lerobot_overview` | Qué es LeRobot y por qué existe, sus piezas, el ciclo de trabajo, cómo piden datos las distintas políticas, la configuración y el Hub |
| `L2_lerobot_dataset` | Por qué el formato tiene esa forma, cada archivo por dentro, `LeRobotDataset`, ventanas de tiempo, aumentación, edición, y la trampa de `episodes=` |
| `L4_normalization_and_processors` | Por qué normalizar, los cuatro modos, el pipeline de procesadores, qué se rompe sin ellos y un bug de 0.4.1 en las estadísticas de imagen |
| `L5_training` | El ciclo de entrenamiento de `lerobot-train` armado a mano, la prueba de memorizar, el comando real y su salida, costos y perillas |
