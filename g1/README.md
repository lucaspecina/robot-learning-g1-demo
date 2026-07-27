# G1: locomoción funcionando

El G1 camina en Isaac Sim comandado por `/cmd_vel`, con la policy pre-entrenada
oficial de Unitree. Sin entrenar nada.

```bash
bash run_g1.sh policy     # arranca el robot con la policy de locomoción
bash run_g1.sh stand      # sin policy: solo sostiene la pose (diagnóstico)
bash run_g1.sh status     # cómo va (factor de tiempo real, altura, comando)
bash run_g1.sh stop

# manejarlo (desde el contenedor jetson o cualquier nodo ROS 2)
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"
```

Validado: 3.2 m caminados manteniendo la altura de pie (0.77-0.78 m).

## La arquitectura

```
              [ agente / misión ]
                      | acciones ROS 2
              [ skills: go_to, pick, ... ]
                      |
                  /cmd_vel          <- ACÁ está la modularidad
                      |
    +-----------------+--------------------------------+
    |  EL ROBOT (g1_robot.py)                          |
    |   física + locomoción, lazo de control cerrado    |
    |   la locomoción es una clase intercambiable       |
    |   (locomotion.py: RLPolicy / StandStill / ...)    |
    +-----------------+--------------------------------+
                      | /g1/odom, /g1/joint_states
```

| Topic | Mensaje | Contenido |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | velocidad deseada del cuerpo |
| `/g1/odom` | `nav_msgs/Odometry` | posición, orientación y velocidades |
| `/g1/joint_states` | `sensor_msgs/JointState` | ángulos y velocidades de las 12 articulaciones |

**Por qué física y locomoción van en el mismo proceso**: un bípedo necesita que
la decisión de la policy y el paso de física estén sincronizados. Separarlos en
procesos asíncronos abre el lazo en el tiempo — el robot ejecuta órdenes
calculadas para un estado viejo, con retraso variable — y se cae. Lo probamos:
así no se sostiene. En el robot real pasa lo mismo, por eso la policy vive
pegada al hardware y no del otro lado de una red.

La modularidad está donde el robot real también la tiene: en `/cmd_vel`. Con el
robot físico, este bloque se reemplaza por el driver de Unitree —con su
locomoción de fábrica o con la nuestra cargada en la Jetson— y el agente, las
skills y la navegación no se enteran.

## Las cuatro cosas que había que acertar (y costaron)

**1. El robot tiene que ser el que la policy conoce.** La policy de
`unitree_rl_gym` fue entrenada sobre el modelo G1 de 12 articulaciones de
Unitree: 32 kg, solo piernas, brazos como masa fija. El G1 que trae IsaacLab es
otro robot (37 articulaciones, brazos y torso móviles). Una policy aprende la
dinámica de UN cuerpo; puesta en otro se cae, y ningún ajuste de ganancias lo
arregla. Solución: convertir el URDF original de Unitree a USD con la
herramienta de IsaacLab (`scripts/tools/convert_urdf.py`) y usar ese modelo
(ver `g1_asset.py`).

**2. Física a 500 Hz.** Con pasos más gruesos (200 Hz) los contactos pie-piso se
integran mal y el robot se desestabiliza. Es la misma lección que aprendimos
debugueando el temblor del Go2, y en un bípedo pesa mucho más. El despliegue
oficial de Unitree usa 500 Hz: hay que respetarlo.

**3. Escribir el estado inicial explícitamente.** Después de crear la escena hay
que escribirle al robot su pose de arranque (`write_root_pose_to_sim`,
`write_joint_state_to_sim`). Sin eso el articulado nace colapsado en el piso por
más que su configuración declare que está de pie. Es un paso estándar de
IsaacLab que es fácil saltear escribiendo un script desde cero.

**4. Nada de "asentamiento" con las piernas rígidas.** Un bípedo con las
articulaciones fijas en una pose no se sostiene: no hay polígono de apoyo que lo
perdone como al cuadrúpedo. "Estar parado" ya requiere control activo, así que
la policy tiene que tomar el control desde el primer instante.

## La policy

`motion.pt` de [unitree_rl_gym](https://github.com/unitreerobotics/unitree_rl_gym)
(`deploy/pre_train/g1/`), la referencia oficial de despliegue de Unitree.

- **Entrada (47 números)**: velocidad angular del cuerpo, dirección de la
  gravedad vista desde el cuerpo (inclinación), comando de velocidad, 12
  ángulos, 12 velocidades articulares, las 12 acciones anteriores, y dos números
  de fase del paso (seno y coseno de un ciclo de 0.8 s).
- **Salida (12 números)**: desviaciones respecto de la pose nominal, una por
  articulación de pierna.
- **No controla los brazos** — en este modelo ni siquiera existen como
  articulaciones. Para la demo, el brazo del G1 va a necesitar su propio
  controlador y un modelo con brazos; ahí habrá que decidir entre entrenar una
  policy de cuerpo completo (el entorno existe en IsaacLab y en
  [unitree_rl_lab](https://github.com/unitreerobotics/unitree_rl_lab)) o
  desacoplar piernas y brazos.

Toda su configuración vive en [config/g1_locomotion.yaml](config/g1_locomotion.yaml).

## Los otros dos caminos que existen (para cuando toque entrenar)

| Repo oficial | Simulador | Modelo | Trae policy |
|---|---|---|---|
| `unitree_rl_gym` | IsaacGym (entrenar), MuJoCo (validar) | G1 de 12 art. | **sí** — la que usamos |
| `unitree_rl_lab` | Isaac Sim / IsaacLab 2.3 | G1 de 29 art. | no, hay que entrenar |

El flujo oficial de Unitree es: entrenar → verificar → sim2sim en MuJoCo →
robot real.

## Rendimiento

El simulador corre a un factor de tiempo real de ~0.14 en la T4 (14 % de la
velocidad real): un humanoide de 12 articulaciones a 500 Hz de física es caro.
No afecta a la locomoción, porque física y policy comparten el lazo y su
relación es exacta. **Sí va a importar** cuando entren nodos que corren en
tiempo real (navegación, agente): ahí hay que usar el reloj simulado de ROS 2
(`/clock` + `use_sim_time`) para que todo el sistema viva en tiempo simulado.
