# notebooks/

Cada paso del flujo, para correrlo y mirarlo. La lógica vive en
[`toolkit/`](../toolkit/); los notebooks la llaman y grafican.

| Notebook | Qué se ve |
|---|---|
| `01_raw_recordings` | Qué trae una grabación de teleoperación: estado, acción, cámara, duraciones |
| `02_convert_to_lerobot` | La conversión a LeRobot, cómo pide los datos ACT y qué se pierde al comprimir |
| `03_which_data_trained_act` | Cómo se reconstruyó con qué episodios se entrenó la primera ACT |

Se abren con el entorno del repo (`bash scripts/setup_learning.sh`):

```bash
source .venv/bin/activate
jupyter lab notebooks/
```

Los notebooks se versionan sin salidas; al correrlos, las salidas quedan sólo
en tu copia.
