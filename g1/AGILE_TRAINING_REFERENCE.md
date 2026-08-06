# AGILE: el taller de entrenamiento de policies de NVIDIA

Fecha de lectura: 2026-08-05. Fuente primaria:
[arXiv:2603.20147v1](https://arxiv.org/html/2603.20147v1), *AGILE: A
Comprehensive Workflow for Humanoid Loco-Manipulation Learning*, Zhao,
Cathomen, Gulich, Liu, Ongan, Lin, Jain, Pouya y Chang (NVIDIA, marzo 2026).
Código: [nvidia-isaac/WBC-AGILE](https://github.com/nvidia-isaac/WBC-AGILE),
clonado en la VM en `~/go2-lab/WBC-AGILE`, commit `7259792`.

## Para qué existe este documento

Hoy el proyecto **consume** una policy de AGILE: usamos el TorchScript *student*
`unitree_g1_velocity_height_recurrent_student.pt` con su descriptor YAML, y no
entrenamos nada. Eso es correcto y es exactamente el uso que AGILE previó para
su etapa de despliegue.

Pero de las cuatro etapas del flujo —preparar, entrenar, evaluar, desplegar—
sólo usamos la última. Cuando toque entrenar (manipulación primero, y quizá un
ajuste fino del balance), este documento es el mapa para no empezar de cero.

AGILE significa **"A Generic Isaac-Lab based Engine"**.

## Qué es AGILE y qué no es

AGILE es un **flujo de ingeniería para fabricar y validar programas de control
de movimiento**. Un "programa de control" acá es una red neuronal que recibe el
estado del cuerpo y devuelve ángulos objetivo para las articulaciones, 50 veces
por segundo.

Los autores dicen atacar dos problemas concretos del estado del arte:

1. **La brecha de flujo de trabajo**: el desarrollo es "fragmentado y ad hoc".
   Errores como un eje de articulación invertido o un término de recompensa mal
   escrito "se descubren frecuentemente sólo después de ciclos de entrenamiento
   costosos".
2. **La brecha de transferencia**: exportar una policy aprendida para validarla
   afuera o desplegarla en hardware es "notoriamente frágil". Sin un contrato de
   entrada/salida estandarizado hay que resolver a mano el orden de las
   articulaciones, reconstruir los búferes de historia y alinear la escala de
   las acciones.

**Lo que AGILE explícitamente no cubre**, según su propia sección de
limitaciones: percepción, mapas, SLAM, navegación autónoma, planificadores
basados en modelos de lenguaje y razonamiento de alto nivel. Sus tareas son
"principalmente propioceptivas" —el robot se siente a sí mismo, no mira el
mundo— y sus comandos son velocidad, altura de pelvis o una secuencia grabada.
Esa frontera es justamente donde empieza nuestro sistema.

## Las cuatro etapas

### 1. Prepare — validar antes de gastar GPU

Tres herramientas gráficas dentro de Isaac Lab, pensadas para atrapar en minutos
errores que de otro modo aparecen después de diez horas de entrenamiento:

- **GUI de posición articular**: una barra deslizante por articulación con
  lectura de torque en vivo. Su *modo simetría* muestra dos robots espejados
  lado a lado, que es como se descubren los errores de signo en los ejes de
  alabeo y guiñada.
- **GUI de manipulación de objetos**: coloca un objeto en las seis dimensiones
  y dibuja los sensores de contacto, para comprobar que las recompensas de
  manipulación se disparan cuando deben.
- **Visualizador de recompensas**: superpone cada término de recompensa con su
  peso y su contribución mientras uno mueve la escena a mano.

La primera buena práctica de los autores es **validar el modelo USD del robot
antes de cualquier entrenamiento**. Nuestra propia historia lo confirma desde el
otro lado: la campaña de deriva de 9 cm/s documentada en
[`LOCOMOTION_EVALUATION.md`](LOCOMOTION_EVALUATION.md) fue exactamente el tipo
de error que esta etapa existe para prevenir.

### 2. Train — reproducibilidad por defecto

Base: Isaac Lab con RSL-RL, algoritmo PPO, **4096 entornos en paralelo**.

Cada corrida guarda automáticamente una instantánea liviana de git —hash del
commit, rama y los cambios sin commitear— junto con un volcado YAML de toda la
configuración. La orquestación va por Docker y Weights & Biases.

Para explorar hiperparámetros existen los **"scaled-dict parameters"**: en vez
de barrer por separado cada entrada de un grupo estructurado (por ejemplo las
ganancias PD de las piernas), se define un único escalar que multiplica el
diccionario entero. Colapsa la búsqueda a una dimensión conservando las
relaciones internas entre articulaciones.

### 3. Evaluate — dos regímenes que miden cosas distintas

Esta es la etapa que más nos conviene copiar, porque ya llegamos sin querer a
la mitad de sus conclusiones.

- **Escenarios determinísticos**: todos los entornos paralelos reciben la misma
  secuencia de comandos guionada (un barrido de velocidades, una rampa de
  altura). Dan "referencias reproducibles de baja varianza para pruebas de
  regresión".
- **Rollouts estocásticos**: comandos remuestreados al azar cada 2 s. Miden
  robustez, pero necesitan mucho más tiempo para converger: los autores usan
  200 s y 500 s, y reportan media ± desvío sobre 10 ejecuciones.

Y sobre todo, **métricas de calidad de movimiento por articulación**:

| Métrica | Qué detecta |
|---|---|
| Aceleración RMS articular | movimiento brusco |
| Jerk RMS articular | sacudidas; es la derivada de la aceleración |
| Violaciones de límites articulares | pide al motor más de lo que puede |
| Fracción de energía por encima de 10 Hz | vibración de alta frecuencia |

Los autores son tajantes con la última: **las violaciones sostenidas de límites
articulares impiden de forma confiable la transferencia sim-to-sim**. Es una
señal de rechazo, no un detalle estético.

Vale la pena registrar la coincidencia: nuestro `checks.py walk` mide balanceo
lateral y frontal, velocidad angular y rebote vertical porque una inspección
visual tuya rechazó una caminata que los números aprobaban. Llegamos a la misma
familia de métricas por el camino difícil. Adoptar las cuatro de AGILE cierra
el hueco que queda, que es el jerk y la energía de alta frecuencia.

### 4. Deploy — el contrato de entrada/salida

Cada policy se exporta como **TorchScript más un descriptor YAML autogenerado**
que captura el contrato completo: nombres de las articulaciones, orden de las
observaciones, búferes de historia y escalado de las acciones.

Tres artefactos distintos, que no hay que confundir:

| Archivo | Para qué |
|---|---|
| `*_checkpoint.pt` | entrenamiento y depuración; **no** es para desplegar |
| `*.pt` (TorchScript) | inferencia sin Python, el que va al robot |
| `*.yaml` | el contrato de entrada/salida |

El mismo descriptor lo relee el canal sim-to-sim de MuJoCo para reconstruir
sola la observación, el mapeo de acciones y los búferes. Y las integraciones
con drivers de hardware reutilizan ese contrato: "sólo cambia el proveedor de
estado", la lógica de inferencia queda idéntica.

Esto explica por qué `run_g1.sh` verifica el commit exacto de WBC-AGILE y por
qué en [`LOCOMOTION_EVALUATION.md`](LOCOMOTION_EVALUATION.md) quedó escrito que
copiar solamente un `.pt` repetiría el error: **el `.pt` sin su YAML no es una
policy, es medio contrato**.

## La arquitectura desacoplada: dónde encastra nuestro sistema

El diseño central para loco-manipulación separa deliberadamente:

- **Cuerpo inferior**: una policy de RL controla las 10–12 articulaciones de las
  piernas y se ocupa de locomoción y equilibrio.
- **Cuerpo superior**: reservado para controladores independientes, sea
  cinemática inversa o un modelo visión-lenguaje-acción.

Durante el entrenamiento las articulaciones superiores reciben objetivos
aleatorios mediante perfiles de velocidad trapezoidales, "para mantener
disponibilidad de grados de libertad". En despliegue, **"la policy congelada de
locomoción sirve como API del cuerpo inferior"** mientras un experto separado
maneja arriba.

Esa frase describe literalmente lo que ya tenemos: policy AGILE en las piernas
más `pink_arm_control.py` en los brazos. La costura ya está en su lugar, y es
por donde entrará el agarre.

## Teacher y student: qué información se descarta a propósito

AGILE entrena primero un **maestro** con información privilegiada que el robot
real nunca va a tener:

- escaneos del terreno que viene;
- fuerzas y estados de contacto verdaderos;
- velocidades verdaderas del cuerpo, sin integrar un acelerómetro ruidoso;
- pose exacta del objeto y distancia mano-objeto, en manipulación.

Después **destila** ese maestro a un **estudiante** que sólo usa lo que existe
en hardware: unidad inercial, encoders de las articulaciones y un contacto
binario simple. El costo de la destilación está medido (barrido determinístico
de 50 s en MuJoCo, tarea de locomoción con altura):

| | Maestro | Estudiante RNN | Estudiante MLP con historia |
|---|---:|---:|---:|
| Error en `vx` | 0,070 m/s | 0,116 m/s | 0,097 m/s |
| Error en `vy` | 0,083 m/s | 0,110 m/s | 0,087 m/s |
| Error en `ωz` | 0,074 rad/s | 0,117 rad/s | 0,079 rad/s |
| Error de altura | 0,035 m | — | 0,037 m |

Esto explica una decisión que ya habíamos tomado y documentado en
[`LOCOMANIPULATION_REFERENCE.md`](LOCOMANIPULATION_REFERENCE.md): descartamos
`agile_locomotion.pt` como candidata de despliegue porque su entorno la conecta
a observaciones de maestro, que incluyen la velocidad lineal verdadera tomada
del simulador. La policy que usamos es la versión estudiante, y por eso es la
correcta.

## Las cinco mejoras algorítmicas, y cuándo sirve cada una

Se activan por separado desde la configuración. No son magia: los autores
insisten en que "cada una debe evaluarse empíricamente para el problema
concreto".

**L2C2 — continuidad Lipschitz local.** Penaliza que la policy cambie mucho
ante entradas parecidas, evaluando en un punto interpolado entre dos estados
consecutivos. Con `λπ = 1.0` y `λV = 0.1`. Reduce jerk, violaciones de límites
y energía de alta frecuencia, y el beneficio crece con el ruido. Los autores
anotan que sin L2C2 "los actuadores producen oscilaciones audibles de alta
frecuencia". **Es la candidata directa contra el balanceo que rechazaste.**

**Normalización de recompensas en línea.** Divide la recompensa por un desvío
estándar estimado con media móvil, corregido por el factor de varianza del
retorno descontado. Hace que el entrenamiento sea invariante a cambios de escala
de las recompensas durante el currículum. En la ablación, a escala 100× sin
normalizar el rendimiento se degrada; con normalización se recupera.

**Terminaciones con arranque de valor.** Corrige un problema perverso: si el
castigo por caerse es menor que el valor esperado de seguir, al agente le
conviene morirse. La solución agrega `γV(x_T)` más un término fijo `σ = 5` con
signo según la terminación sea mala (caída), buena (llegar al objetivo) o neutra
(fin de tiempo). Al ser invariante de escala, ese `σ = 5` sirve para todas las
tareas sin ajuste.

**Arnés virtual.** Fuerzas PD externas aplicadas al cuerpo raíz que sostienen al
robot durante el entrenamiento temprano, "igual que un arnés físico sostiene a
una persona que aprende a caminar". Un factor `s ∈ [0,1]` multiplica todas las
ganancias y decae con uno de tres calendarios: lineal, exponencial, o adaptativo
—este último baja `s` sólo cuando la proporción de robots parados supera un
umbral—. Imprescindible para tareas de recuperación como pararse desde el suelo.

**Aumento por simetría.** Espeja observaciones y acciones para fomentar marcha
simétrica y duplicar los datos. El mapeo es configurable, no por índice. La
mejora en recompensa es modesta; el beneficio real es la simetría del
comportamiento, que la curva de recompensa no captura.

## Costos e hiperparámetros

Base PPO común a todas las tareas:

| Parámetro | Valor | Excepciones |
|---|---|---|
| Red del actor | `[256, 256, 128]` | pick & place: `[256, 128, 64]` |
| Red del crítico | `[512, 256, 128]` | pick & place: `[256, 128, 64]` |
| Activación | ELU | — |
| Tasa de aprendizaje | `1e-3` | — |
| Descuento γ | 0,99 | pararse: 0,995 |
| GAE λ | 0,95 | — |
| Razón de recorte | 0,2 | — |
| Minilotes | 4 | — |
| Épocas de aprendizaje | 5 | — |
| Coeficiente de entropía | 0,005 | altura: 0,01; pararse: 0,0025 |
| Entornos | 4096 | — |

Tiempos declarados **sobre una sola GPU L40**:

| Tarea | Costo |
|---|---|
| Locomoción básica | 10 h (20 mil iteraciones) |
| Locomoción con control de altura | 10 h |
| Pararse desde el suelo | 15–25 h |
| Imitación de movimiento | 6 h |
| Tomar y colocar | 10 h |

Nuestra Tesla T4 **no es comparable a una L40**. Antes de planificar un
entrenamiento hay que medir el costo real en la GPU que usemos, o alquilar una
máquina acorde. Este dato también pesa en la decisión pendiente de migrar de
VM, que hoy está motivada por otra razón (Motion BVH del LiDAR).

## Lo que ya está clonado en la VM

Tareas registradas en `~/go2-lab/WBC-AGILE/agile/rl_env/tasks/`:

```
Velocity-G1-History-v0            Velocity-T1-v0
Velocity-Height-G1-v0             HeightTracking-G1-v0
Velocity-Height-G1-Distillation-Recurrent-v0
Velocity-Height-G1-Distillation-History-v0
StandUp-T1-v0                     Tracking-Flat-G1-v0
G1-PickPlace-Tracking-v0          G1-PickPlace-Tracking-v0-Record
G1-PickPlace-Tracking-v0-GR00T-Inference
Debug-G1-v0                       Debug-G1-Object-v0    Debug-T1-v0
```

Comandos base, del README oficial:

```bash
python scripts/train.py --task Velocity-T1-v0 --num_envs 2048 --headless
python scripts/eval.py  --task Velocity-T1-v0 --num_envs 32 --checkpoint <ruta>
```

Y los scripts que corresponden a cada etapa:

| Script | Etapa |
|---|---|
| `scripts/train.py` | entrenar |
| `scripts/eval.py`, `scripts/play.py` | evaluar |
| `scripts/sim2mujoco_eval.py`, `sim2mujoco_watcher.py` | validación sim-to-sim |
| `scripts/export_IODescriptors.py` | generar el contrato de despliegue |
| `scripts/data_recording/` | recolectar demostraciones y convertirlas a GR00T |
| `scripts/wandb_sweep/` | barridos de hiperparámetros |
| `workflows/*.yaml` | orquestación en Docker |

El repositorio trae además `LESSONS_LEARNED.md` y `OFFICE_HOUR_FAQ.md`, que
conviene leer antes del primer entrenamiento.

## El camino oficial para el agarre

Nuestra capacidad `grasp_object` es hoy un `placeholder`. La tarea 5 del paper
es el camino declarado para completarla, y es reproducible con lo que ya está
clonado:

1. **Experto por RL**: una policy controla **solamente brazo derecho y
   cintura**, guiada por trayectorias de referencia y estado privilegiado del
   simulador (pose del objeto, distancia mano-objeto). La locomoción queda
   **congelada** como API del cuerpo inferior. Diez horas en una L40.
2. **Recolección sin humanos**: ese experto genera **100 trayectorias exitosas**
   por simulación paralela con aleatorización de física y de apariencia,
   produciendo tríos de imagen RGB, propiocepción y acciones. No hay
   teleoperación.
3. **Ajuste fino del modelo visión-lenguaje-acción**: se afina **GR00T N1.5**
   sobre esos datos sintéticos. La entrada privilegiada se reemplaza por RGB más
   una instrucción en lenguaje natural.
4. **Evaluación en lazo cerrado**: **90 % de éxito en 100 casos de prueba** con
   estados iniciales del robot muestreados al azar, en simulación.

Dos advertencias que el paper deja explícitas. Primero, esos resultados son
**en simulación**: pick & place es la única de las cinco tareas que no reportan
transferida a hardware. Segundo, el 90 % se mide con el objeto y la escena de su
entorno; no es una promesa sobre nuestra mesa ni nuestro cilindro.

## Cómo esto toca nuestros tres bloqueos

| Bloqueo nuestro | Qué ofrece AGILE |
|---|---|
| Balanceo lateral de 8–10° al caminar, rechazado visualmente | L2C2 contra el jerk y la vibración; y sobre todo, entrenar con la pose de brazos y la carga reales en vez de esperar que la policy generalice |
| `grasp_object` sin implementar | la tarea 5 completa: experto RL, 100 demostraciones sintéticas y ajuste de GR00T N1.5 |
| Localización móvil por la T4 | **nada** — AGILE no toca percepción ni mapas; ese problema es nuestro y se resuelve con GPU o con Point-LIO en el robot real |

El tercero importa porque marca el límite: no todo se arregla con el taller de
entrenamiento.

Sobre el primero conviene ser preciso. `LOCOMOTION_EVALUATION.md` dice que esta
policy fue entrenada para caminar con la parte superior neutral, y por eso la
pose `transporte` la saca de la condición aprendida. `PAYLOAD_TEST_PLAN.md` ya
listaba "continuar el entrenamiento de la policy con pesos, poses de brazos y
cargas laterales variables" como último recurso, después de acercar la carga al
torso y de suavizar los giros. **AGILE es exactamente el taller para ese último
recurso**, y ahora sabemos cuánto cuesta.

## Buenas prácticas que los autores destilan

De su apéndice, y valen igual para nosotros aunque todavía no entrenemos:

1. Validar el modelo USD del robot antes de cualquier entrenamiento.
2. Usar las interfaces de depuración para atrapar errores de configuración en
   minutos, no después de horas de GPU.
3. **Mirar el video de los rollouts: los gráficos engañan.**
4. Probar al menos cinco semillas antes de sacar conclusiones.
5. Priorizar aleatorización de dominio y regularización de acciones para la
   transferencia a hardware.
6. Estructurar las recompensas en tres grupos: **tarea** (qué lograr),
   **estilo** (cómo debe verse) y **regularización** (qué evitar).

La tercera es la misma conclusión a la que llegamos acá por otro camino, y está
escrita en [`AGENTS.md`](../AGENTS.md): un resultado numérico no valida por sí
solo que el movimiento tenga sentido.

## Límites declarados por los autores

- Validado en **dos plataformas**: Unitree G1 y Booster T1. Más cobertura de
  hardware queda como trabajo futuro.
- Depende de las APIs de Isaac Lab, que evolucionan.
- Las tareas son **principalmente propioceptivas**; manipulación guiada por
  percepción y locomoción más dinámica —correr, subir escaleras— no están
  incluidas.
- **No hubo captura de movimiento disponible**: la transferencia a hardware se
  valida cualitativamente con demostraciones en video, y las métricas
  cuantitativas salen del canal de MuJoCo, no del robot real.

Nota de correspondencia: el paper describe el G1 con 23 grados de libertad
(12 de piernas y 11 de brazos y torso), mientras nuestra integración usa el
cuerpo de 29. Antes de reproducir cualquier tarea hay que confirmar contra qué
configuración exacta está escrita.

## Fuentes

- [AGILE, paper completo](https://arxiv.org/html/2603.20147v1)
- [WBC-AGILE, repositorio oficial](https://github.com/nvidia-isaac/WBC-AGILE)
- [WBC-AGILE, documentación de tareas](https://nvidia-isaac.github.io/WBC-AGILE/tasks.html)
- [Isaac Lab](https://github.com/isaac-sim/IsaacLab)
- [RSL-RL, la implementación de PPO que usa](https://github.com/leggedrobotics/rsl_rl)
- [GR00T N1.5](https://github.com/NVIDIA/Isaac-GR00T)
