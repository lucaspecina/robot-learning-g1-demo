# toolkit/

Herramientas reusables con cualquier política y en cualquier máquina (Mac, VM,
Jetson). El código vive en módulos con tests; los notebooks sólo los llaman y
grafican.

## Instalación

```bash
bash scripts/setup_learning.sh     # clona unitree_lerobot fijado e instala con uv
source .venv/bin/activate
pytest                             # tests del toolkit
```

Los datasets y modelos van a `data/`, que Git ignora.

## Herramientas

| Módulo | Pregunta que responde | Estado |
|---|---|---|
| `dataset/convert_unitree` | Grabaciones de teleoperación → dataset de LeRobot | listo |
| `dataset/stats` | ¿Qué estadísticas tendría un entrenamiento sobre estos episodios? | listo |
| `checks/normalization` | ¿El dataset y la política comparten la normalización? | listo |
| `dataset/inspect` | ¿Los episodios están sanos? Duración, saltos, cámaras | por traer del notebook de ACT |
| `policy/load` | Cargar una política con su pre y postprocesador | listo |
| `policy/act_internals` | ACT desarmado en sus piezas (encoder del VAE, `z`, tokens, encoder, decoder, atención) para mirarlas | listo |
| `eval/offline` | ¿Predice bien sobre episodios que no vio? Error por articulación y horizonte, contra referencias | listo |
| `eval/rollout_sim` | ¿Cumple la tarea cuando sus propios errores se acumulan? | por hacer |
| `eval/dry_run` | ¿Qué haría en el robot sin moverlo? | por traer de `eval_g1.py` |
| `validation/*` | Chequeos de datos por categoría (ver `docs/08_validacion_de_datos.md`) | listo |
| `compare` | ¿Esta política es mejor que la anterior, con las mismas métricas? | por hacer |

## Ejemplos

```bash
# convertir una tarea grabada
python -m toolkit.dataset.convert_unitree \
    --raw-dir ~/Desktop/episodios/pick_mameluco --repo-id local/pick_mameluco

# comparar contra la normalización de una política entrenada
python -m toolkit.checks.normalization \
    --dataset-root data/datasets/local/pick_mameluco \
    --policy-dir data/checkpoints/act_cheap_v1/pretrained_model
```

## Trampas conocidas

- **`LeRobotDataset(..., episodes=[...])` no filtra filas en LeRobot 0.4.1**
  cuando el dataset es local o todos sus episodios comparten archivo: sólo
  decide qué archivos bajar del Hub. Para separar entrenamiento y evaluación,
  partir el dataset con `lerobot.datasets.dataset_tools.split_dataset`.
  Detalle en `notebooks/learn/L2_lerobot_dataset.ipynb`.
- **El desvío de las imágenes en `stats.json` es falso en LeRobot 0.4.1**:
  calcula sobre `uint8` y el desvío por episodio sale 0 (corregido en la rama
  principal de LeRobot). No afecta mientras se entrene con
  `use_imagenet_stats=true` (el default). Detalle en
  `notebooks/learn/L4_normalization_and_processors.ipynb`.

## Notas de plataforma

**El repo está en el Escritorio, que iCloud sincroniza.** Eso rompió el
entorno dos veces: iCloud marcaba como ocultos archivos `.pth` (Python 3.10
ignora los `.pth` ocultos → `ModuleNotFoundError: toolkit`) y creó copias en
conflicto ("`... 2.pth`") que dejó en la nube, sin contenido local: Python se
colgaba al arrancar intentando leerlas. Arreglo (28-oct-2026): iCloud ignora
las carpetas terminadas en `.nosync`, así que el entorno y los datos viven en
`.venv.nosync/` y `data.nosync/`, y `.venv` y `data` son accesos directos a
ellas. Aun así, algo más en la Mac vuelve a marcar los `.pth` como ocultos
(causa no encontrada). Por eso `setup_learning.sh` escribe además un
`sitecustomize.py` en el entorno, que Python carga siempre al arrancar y que
agrega el repo y `unitree_lerobot` a la ruta sin depender de los `.pth`. Si
igual aparece el error: volver a correr `setup_learning.sh` y buscar archivos
"`* 2.*`" dentro del entorno.

En la Mac, los videos se leen con `video_backend="pyav"`. El decodificador por
defecto de LeRobot (torchcodec) necesita bibliotecas de FFmpeg para arm64, y el
FFmpeg instalado con Homebrew en esa máquina es de Intel.

La evaluación offline usa *teacher forcing*: en cada paso la política recibe la
observación verdadera, no la que resultó de sus propias acciones. Mide si
aprendió, pero no si la tarea sale; para eso está el rollout en lazo cerrado.
