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
| `L3_act_explained` | Por qué ACT tiene la forma que tiene, qué espera y devuelve, una predicción capa por capa, el VAE de nuestro modelo y cómo ejecutar los bloques |
| `L1_lerobot_overview` | Qué es LeRobot y por qué existe, sus piezas, el ciclo de trabajo, cómo piden datos las distintas políticas, la configuración y el Hub |
