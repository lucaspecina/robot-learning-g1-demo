# Receta para teleoperar el G1 simulado con el Quest 3

> Escrita el 10-ago-2026, **antes** de tener la máquina con placa adecuada. El
> objetivo es que el día que llegue sea seguir pasos y no volver a investigar.
>
> El flujo que seguimos está en
> [`05_workflow_de_entrenamiento.md`](05_workflow_de_entrenamiento.md); esto es
> el cómo operativo.

## Lo que ya está verificado en la VM actual

| Pieza | Estado |
|---|---|
| Adaptadores de mandos de Quest para el G1 | **presentes** en las dos instalaciones de Isaac Lab (`.../devices/openxr/retargeters/humanoid/unitree/`), con control de locomoción, de la mano de tres dedos y de la Inspire |
| Tarea `Isaac-PickPlace-Locomanipulation-G1-Abs-v0` | **registrada** |
| Scripts `record_demos.py`, `annotate_demos.py`, `generate_dataset.py` | **presentes** |
| Isaac Lab Arena | **clonado** el 10-ago con sus submódulos |
| Credenciales de NGC | **faltan** |
| Placa con generación Ada o Blackwell | **falta** — es el único bloqueo real |

## Fricciones que ya encontramos, para no repetirlas

1. **Arena hay que clonarlo con submódulos.** Trae Isaac Lab y GR00T adentro.
   Un clon común deja las carpetas vacías.
2. **Los submódulos apuntan a direcciones SSH de GitHub.** Si la máquina no
   tiene clave configurada, fallan. Se resuelve reescribiéndolas a HTTPS:

   ```bash
   git config --file .gitmodules --get-regexp url | while read k v; do
     git config --file .gitmodules "$k" "$(echo "$v" | sed 's|git@github.com:|https://github.com/|')"
   done
   git submodule sync && git submodule update --init --depth 1
   ```
3. **Hace falta `uv`**, que no viene instalado:
   `curl -LsSf https://astral.sh/uv/install.sh | sh`
4. **Arena no reutiliza el Isaac Sim del sistema.** Crea su propio entorno con
   `uv sync`, con sus versiones de Isaac Sim, PyTorch y **Newton** — el motor de
   física nuevo. Apunta a los requisitos de **Isaac Sim 6.0**, no la 5.1.
5. **El driver de la VM actual (570.195) está por debajo del mínimo de CloudXR**
   (573.42). En la máquina nueva hay que subirlo.

## Los pasos, cuando esté la máquina

### 1. Preparar el entorno

```bash
git clone https://github.com/isaac-sim/IsaacLab-Arena.git
cd IsaacLab-Arena
# reescribir submódulos a HTTPS (ver arriba) y bajarlos
uv sync
source .venv/bin/activate
export OMNI_KIT_ACCEPT_EULA=YES ACCEPT_EULA=Y
```

Verificación mínima, sin robot ni casco:

```bash
python isaaclab_arena/evaluation/policy_runner.py \
  --policy_type zero_action --num_steps 20 cube_goal_pose
```

### 2. Levantar CloudXR

Corre como contenedor en la misma máquina y transmite al casco.
**Requiere credenciales de NGC** para bajar la imagen.

Puertos a abrir: **49100/tcp** (señalización), **47998/udp** (video),
**48322/tcp** (proxy).

### 3. Conectar el casco

En el Quest 3 **no se instala nada**: se abre el cliente web de Isaac Teleop en
el navegador del casco. La conexión es por red local y la documentación pide
router de wifi moderno.

Para desarrollar sin ponerse el casco existe un emulador que hace que el
navegador de escritorio se haga pasar por un Quest 3 — sirve para depurar, pero
**no evita necesitar la placa**, porque emula el cliente y no el servidor.

### 4. Grabar

```bash
python scripts/tools/record_demos.py \
  --task <tarea del G1> \
  --teleop_device <dispositivo XR> \
  --dataset_file ./datasets/<nombre>.hdf5 \
  --num_demos 10
```

**Diez demostraciones alcanzan** para que la herramienta de multiplicación
funcione. No hay que grabar doscientas a mano.

Antes de grabar en masa, **convertir una grabación corta y revisarla**. Es el
consejo más barato del flujo: evita descubrir que todo el lote es inservible.

### 5. Multiplicar, exportar y afinar

Anotar los límites de subtarea, generar cientos de variantes, convertir al
formato que consume el modelo, y afinar. Detalle en
[`04_generacion_de_datos.md`](04_generacion_de_datos.md).

## Controles, por si se usa teclado en vez de casco

Sirven para probar el flujo sin equipo de realidad virtual:

| Tecla | Efecto |
|---|---|
| `W` `S` | adelante y atrás |
| `A` `D` | a los costados |
| `Q` `E` | subir y bajar |
| `Z` `X` `T` `G` `C` `V` | rotar la muñeca |
| `K` | abrir y cerrar la pinza |
| `L` | reiniciar el intento |

## Lo que trae Arena y conviene mirar

Además de la tarea del curso, tiene módulos que se cruzan directamente con
trabajo que ya hicimos a mano:

- `g1_wbc_agile_pink` — **la combinación de AGILE con Pink que reimplementamos**
  por nuestra cuenta. Vale la pena comparar.
- `g1_agile_standing_test` — prueba de quietud, equivalente a nuestro peldaño.
- `galileo_g1_locomanip_pick_and_place` — la tarea de agarrar y colocar.
- `isaaclab_arena_gr00t` — el puente hacia el modelo, incluida la exportación.
- `isaaclab_arena_cosmos` y `isaaclab_arena_dreamzero` — generación de datos
  sintéticos.
