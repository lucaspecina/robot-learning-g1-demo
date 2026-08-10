# Cómo se arma la máquina de simulación, desde cero

> Escrito el 10-ago-2026, capturando lo que había en `vm-go2-isaac` antes de
> montar una máquina nueva. **Hasta hoy este entorno se había armado a mano y
> nunca se documentó** — era el hueco más grande del proyecto.

## Por qué existe este directorio

Estos scripts vivían sueltos en `~/go2-lab/` del disco de la VM y no estaban en
Git. Si ese disco se perdía, se perdían. Tienen adentro lecciones caras:

- `setsid` al lanzar el simulador, porque sin él un `Ctrl+C` en la sesión SSH
  mataba a Isaac;
- purgar el demonio de ROS 2 antes de arrancar, por publicadores fantasma de
  instancias muertas;
- **no mezclar los dos Python**: el de Isaac es 3.11 y el del sistema 3.12, y el
  puente ROS usa sus propias bibliotecas internas;
- una heurística para diagnosticar pantalla negra mirando la GPU y no la CPU,
  porque la CPU siempre está alta y no discrimina nada.

| Archivo | Qué hace |
|---|---|
| `isaac_ros_env.sh` | variables para que Isaac Sim hable ROS 2 con sus bibliotecas internas |
| `isaac.sh` | arrancar, parar, ver estado y diagnosticar Isaac Sim en modo streaming |

Quedaron afuera por redundantes `isaac_start.sh` e `isaac_stop.sh`, que hacen lo
mismo que `isaac.sh start` y `stop`, y `launch_port.sh`, que pertenece al
simulador portado del Go2 (fase 2) y no a la línea del G1.

---

## Lo que hay hoy en la máquina, para reproducirlo

Capturado de `vm-go2-isaac` el 10-ago-2026:

| Pieza | Versión / ubicación |
|---|---|
| Sistema | Ubuntu 24.04.3 LTS, kernel 6.14 |
| Driver NVIDIA | **570.195.03** |
| Isaac Sim | **5.1.0-rc.19**, en `/opt/IsaacSim` (imagen de Marketplace de NVIDIA) |
| Isaac Lab | rama de desarrollo, commit `858234d0` (19-jul-2026) |
| Isaac Lab de referencia | `IsaacLab-v2.3.2`, en su propia carpeta |
| WBC-AGILE | commit **`7259792`** — verificado por `run_g1.sh`, no cambiar sin repetir pruebas |
| `unitree_rl_gym` | para el URDF de 29 articulaciones y la policy anterior |
| Entornos de Python | `venvs/wbc-agile` y `venvs/wbc-agile-v2.3.2` |
| Capa de compatibilidad | `compat/isaaclab-v2.3.2-numpy1` — Pinocchio necesita NumPy 1 |

Estructura esperada en `~/go2-lab/`:

```
robot-learning-g1-demo/     este repo
g1 -> robot-learning-g1-demo/g1      symlink; lab_sync.sh lo verifica
IsaacLab/  IsaacLab-v2.3.2/  WBC-AGILE/  unitree_rl_gym/
venvs/  compat/  logs/  experiments/
.env                        credenciales de Azure, permisos 600
isaac_ros_env.sh            copiar desde scripts/vm/
```

---

## Pasos para la máquina nueva

1. **Crear la VM.** Ver la especificación en
   [`../../docs/04_generacion_de_datos.md`](../../docs/04_generacion_de_datos.md).
   Puntos que ya nos mordieron: **controlador de disco NVMe** (obligatorio en la
   familia RTX PRO 6000), **zona de disponibilidad 2 o 3** (la 1 no la soporta),
   y disco de **512 GB como mínimo**.
2. **Driver NVIDIA.** El de la VM actual es 570.195, y **CloudXR exige 573.42 o
   posterior**. En una máquina para teleoperación hay que subirlo.
3. **Isaac Sim.** Lo más simple es usar la imagen de Marketplace de NVIDIA, que
   ya lo trae en `/opt/IsaacSim`. Si no está disponible para esa familia, se
   instala a mano.
4. **Clonar los repos** en `~/go2-lab/` según la tabla de arriba, respetando el
   commit exacto de WBC-AGILE.
5. **Crear el symlink** `~/go2-lab/g1`.
6. **Copiar `isaac_ros_env.sh`** desde este directorio.
7. **Recrear el `.env`** con las credenciales de Azure — ver `.env.example` en
   la raíz del repo. Permisos 600.
8. **Levantar los contenedores** con `bash systems/systems_up.sh`.
9. **Regenerar los assets USD**, que están ignorados por Git:
   `IsaacLab/scripts/tools/convert_urdf.py <urdf> assets/<nombre>.usd --headless
   --joint-stiffness 0 --joint-damping 0`.
10. **Verificar** con la escalera: `bash run_demo.sh check all`.

## Para teleoperación con casco, además

CloudXR corre como contenedor en la misma máquina y transmite al casco por red.
Requisitos y puertos en
[`../../docs/04_generacion_de_datos.md`](../../docs/04_generacion_de_datos.md);
el resumen es placa de generación Ada o Blackwell, driver ≥ 573.42, y abrir
49100/tcp, 47998/udp y 48322/tcp.

En el Quest 3 **no se instala nada**: se abre el cliente web de Isaac Teleop en
el navegador del casco.
