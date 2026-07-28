# G1: el sistema de la demo

El robot camina, ve, navega solo y ejecuta misiones dadas en castellano.

```bash
# el robot (necesita GPU: corre Isaac)
bash run_g1.sh policy 29dof 0 "--camera --scene"

# las capas de arriba (dentro del contenedor jetson)
python3 mobility_authority.py  # único dueño de /cmd_vel
python3 stand_hold.py          # corrige la deriva durante una espera
python3 skills/go_to.py       # navegación
python3 skills/detector.py    # percepción
python3 agent/agent.py        # el agente

# darle una misión
ros2 topic pub --once /g1/mission std_msgs/msg/String \
  "{data: fijate la hora en el reloj y llevale la botella a quien corresponda}"
```

## Qué funciona hoy

| Pieza | Estado |
|---|---|
| Locomoción (camina, gira) | funciona — policy pre-entrenada de Unitree |
| Cuerpo con brazos (29 articulaciones) | funciona — camina igual que sin brazos |
| Control de brazos (poses) | funciona — `/g1/arm_pose` |
| Carga en las manos | hasta ~1 kg cómodo, 3 kg al límite |
| Cámara de cabeza | funciona — `/g1/head_cam/image`, 15 Hz |
| Percepción por color | funciona — `/g1/detections` |
| Navegación a un punto | funciona — `/g1/goal` → `/g1/nav_status` |
| Agente con plan y decisión | funciona — 8 de 10 pasos de la misión |
| Agarrar | pendiente (lo hará un VLA) |

**La misión de la demo, ejecutada de punta a punta:**

```
misión: "fijate la hora en el reloj y llevale la botella a quien corresponda"
plan:   ir_a(reloj) → mirar(reloj) → decidir_color → ir_a(mesa) → brazos(listo)
        → mirar(botella) → agarrar → brazos(transporte) → buscar_persona → decir

ir_a(reloj)      OK   navegó solo
mirar(reloj)     OK   lo vio centrado en la imagen
decidir          OK   "el reloj marca 5:00 → persona_roja"
ir_a(mesa)       OK   llegó a la pose de aproximación
brazos(listo)    OK
mirar(botella)   --   no la ve: está sobre la mesa, fuera del cuadro
agarrar          --   pendiente (VLA)
brazos(transporte) OK
buscar_persona   --   la skill no gira todavía: espera pasivamente
```

## La arquitectura

```
                  /g1/mission  (la misión, en castellano)
                        |
              [ agent/agent.py ]     <- SERVIDOR: piensa por evento
                        |
        /g1/goal   /g1/arm_pose   (lee /g1/detections, /g1/nav_status)
                        |
       [ skills/go_to.py ]  [ skills/detector.py ]   <- JETSON
            /g1/cmd_vel/navigation
                        |
     [ stand_hold.py ] -> [ mobility_authority.py ]
                                  |
                              /cmd_vel
                        |
    +-------------------+--------------------------------+
    |  EL ROBOT (g1_robot.py)                            |
    |   física + locomoción, lazo de control cerrado      |
    |   + control de brazos + cámara                      |
    +-------------------+--------------------------------+
              /g1/odom  /g1/joint_states  /g1/head_cam/image
```

| Topic | Mensaje | Quién lo produce |
|---|---|---|
| `/g1/mission` | String | vos (o el reconocimiento de voz, más adelante) |
| `/g1/goal` | PoseStamped | el agente |
| `/g1/nav_status` | String | la navegación |
| `/g1/detections` | String (JSON) | el detector |
| `/g1/arm_pose` | String | el agente |
| `/g1/cmd_vel/{stand,navigation,manual,test}` | Twist | cada fuente identificada |
| `/g1/mobility/request` | String (JSON transitorio) | quien solicita o libera movilidad |
| `/g1/mobility/status` | String (JSON) | el árbitro de movilidad |
| `/cmd_vel` | Twist | sólo `mobility_authority.py` |
| `/g1/odom`, `/g1/joint_states` | Odometry, JointState | el robot |
| `/g1/head_cam/image` | Image | el robot |

Cada pieza se puede reemplazar sin tocar las demás mientras respete su contrato:
la navegación por Nav2, el detector por uno neuronal, el planificador de reglas
por un modelo de lenguaje, el robot simulado por el real.

## Los archivos

| Archivo | Qué hace |
|---|---|
| `g1_robot.py` | el robot: física, locomoción, brazos, cámara. Un proceso, lazo cerrado |
| `locomotion.py` | controladores de locomoción intercambiables (policy RL / solo pararse) |
| `arm_control.py` | control de brazos por poses con nombre |
| `perception.py` | la cámara de la cabeza y su publicación |
| `g1_asset.py` | los cuerpos disponibles (12 y 29 articulaciones) y sus actuadores |
| `demo_scene.py` | la habitación: mesa, botella, reloj, dos personas |
| `skills/go_to.py` | navegación a un punto |
| `mobility_authority.py` | concede la movilidad a una sola fuente y alimenta `/cmd_vel` |
| `stand_hold.py` | mantiene una pose durante una espera; no navega |
| `skills/detector.py` | percepción por color |
| `agent/agent.py` | el agente: plan y ejecución |
| `run_g1.sh` | lanzador |

## Lo que costó acertar (para no repetirlo)

**El robot tiene que ser el que la policy conoce.** La policy de Unitree fue
entrenada sobre el G1 de 12 articulaciones (32 kg, brazos soldados). Puesta en
otro cuerpo se cae, y ningún ajuste de ganancias lo arregla. El modelo se
convierte del URDF original con `IsaacLab/scripts/tools/convert_urdf.py`.

**La cintura va rígida en el cuerpo de 29.** En el modelo de 12 la cintura no
existe. Si en el de 29 se bambolea, el torso se mueve y la policy —que solo
controla piernas— no tiene con qué compensarlo: con la cintura blanda el robot
termina en el piso.

**Física a 500 Hz.** Con pasos más gruesos los contactos pie-piso se integran
mal. Misma lección que el temblor del Go2, peor en un bípedo.

**Escribir el estado inicial explícitamente** después de crear la escena, o el
articulado nace colapsado.

**Nada de "asentamiento" con las piernas rígidas**: un bípedo rígido se cae. La
policy tiene que controlar desde el primer instante.

**Física y locomoción en el mismo proceso.** Separarlas en procesos asíncronos
abre el lazo de control en el tiempo y el bípedo se cae. La modularidad va en
la orden de velocidad. En esta demo, un árbitro exclusivo deja una única salida
en `/cmd_vel`; en el G1 real esa salida podrá alimentar la interfaz de Unitree.

**No se puede esperar dentro de un callback de ROS** ("Executor is already
spinning"): las tareas largas van en un hilo aparte que lee lo que los
callbacks actualizan.

**Los destinos de navegación son poses de aproximación**, no coordenadas del
objeto. Si el destino es el centro de la mesa, el robot camina contra el mueble
y la navegación nunca da por cumplido el objetivo.

**Los plazos medidos en tiempo real no sirven** con el simulador corriendo al
20 %: recorrer 8 m lleva más de dos minutos de reloj de pared. La solución de
fondo es el reloj simulado de ROS 2 (`/clock` + `use_sim_time`).

## Lo siguiente

1. **Reducir la deriva física**: explicar y cerrar la brecha Isaac–MuJoCo sin
   esconderla con `stand_hold.py`.
2. **Reloj simulado** (`/clock` + `use_sim_time`) para medir plazos en tiempo
   de simulación.
3. **Que la cámara vea la mesa**: inclinarla hacia abajo o acercar la pose de
   aproximación. Sin eso no hay nada que agarrar.
4. **`buscar_persona` de verdad**: girar en el lugar hasta tener a la persona
   en el centro de la imagen, en vez de esperar pasivamente.
5. **El agarre** con un VLA entrenado por nosotros.
6. **El planificador con un modelo de lenguaje** (la estructura ya está;
   falta el proveedor y las credenciales).
7. **Leer el reloj de verdad** con un modelo con visión, en vez de la hora fija.
