# Los dos problemas abiertos, y por dónde seguir

Estado al 28-jul. Punto de retorno seguro: etiqueta de git `known-good`
(commit 86f8082) — de pie a 0.72 m, caminó 5.85 m, ciclo congelar/soltar
funcionando.

---

## Problema 1 — `/cmd_vel` no tiene dueño

**El síntoma**: cualquier nodo publica órdenes de velocidad cuando quiere. Si
dos publican a la vez, se anulan y el robot se queda quieto sin que ningún log
diga nada raro. Nos pasó con la navegación manteniendo la posición mientras una
prueba pedía caminar.

**Lo que hay hoy** (paliativo, tres parches sobre el mismo agujero): la skill
de navegación mantiene la posición cuando no tiene objetivo, y un interruptor
`/g1/hold` le pide que suelte el volante.

**El diagnóstico de Codex, que corrige el nuestro**: "quedarse quieto es un
comportamiento activo" es correcto, pero "siempre debe haber navegación
cerrando el lazo de posición" NO lo es. El navegador **no debe publicar cuando
no tiene misión** — Nav2 termina, publica velocidad cero y se calla; sus 25 cm
son tolerancia *de llegada*, no de mantener posición. Confundimos las dos cosas.

**El diseño que corresponde**:

```
PASSIVE/FAULT → STAND_HOLD ↔ NAVIGATE → SETTLE → MANIPULATE
                            (+ REPOSITION para corregir la base en tarea)
```

- Un **único árbitro** dueño de `/cmd_vel`; solo el estado vigente escribe.
- `STAND_HOLD` vive en el supervisor de locomoción, **no** en la navegación, y
  ancla en el marco de odometría (no en el mapa).
- `NAVIGATE` solo manda mientras navega.
- `MANIPULATE` mantiene balance pero **no debe empezar a caminar de sorpresa**
  mientras el brazo toca la mesa o sostiene el objeto. Si se sale del sobre
  permitido: pausar el brazo, asegurar el objeto, reposicionar, y recién
  entonces reintentar.

Siempre activos, en todos los estados: estimación de estado, balance, postura,
watchdog y seguridad.

**Tolerancias iniciales** (de Codex, para esta demo):

| Situación | Corregir si supera | Liberar al bajar de | Velocidad de corrección |
|---|---|---|---|
| Sala abierta | 5 cm o 3° | 2 cm o 1.5° | 0.10–0.15 m/s |
| Antes de manipular | error <2–3 cm y <2°, velocidad <2 cm/s, yaw-rate <0.03 rad/s, sostenido 1–2 s | | |
| Durante el agarre | 5 cm o 5° → **abortar y reposicionar**, nunca corregir en silencio | | |

Los 25 cm sirven como sobre de emergencia, no como mantener posición: con 9 cm/s
de deriva se consumen en 2.8 segundos.

**Precedentes**: Spot tiene un comando de "pararse" explícito que detiene el
movimiento y sostiene una pose respecto de la huella de los pies, separado de
los comandos de trayectoria. Unitree implementa `FixStand` como estado propio
en `unitree_rl_lab/deploy/include/FSM/State_FixStand.h`.

---

## Problema 2 — La deriva en Isaac es 3-4x la de MuJoCo

Con `/cmd_vel = (0,0,0)` y verificado que nadie más publica:

| | Deriva | Forma |
|---|---|---|
| Isaac, cuerpo de 29 art. | **9.0 cm/s** | recta sostenida |
| Isaac, cuerpo de 12 art. | 7.6 cm/s | recta sostenida |
| MuJoCo oficial, mismo `motion.pt` | **2.4 cm/s** | medido dos veces por separado |

### Lo descartado CON MEDICIONES (no repetir)

- Fricción del piso (IsaacLab trae 0.5, MuJoCo usa 1.0; subida, sin efecto)
- Fricción seca y armadura de las articulaciones (0.1 y 0.01 del modelo
  oficial; aplicadas, sin efecto)
- Reset de la memoria LSTM de la policy (bisección con git: inocente)
- Cuerpo de 12 vs 29 articulaciones (los dos derivan)
- Las 47 observaciones y la señal de fase (comparadas campo por campo contra
  `deploy_mujoco.py`: idénticas)
- PD explícito vs implícito de IsaacLab (misma magnitud)
- Inercias distintas entre URDF y MJCF: Codex las trasplantó en MuJoCo y la
  deriva siguió en 2.3 cm/s

### El hallazgo de Codex: reprodujo el factor 3-4x en MuJoCo

Tocando cosas mínimas, sin ejes invertidos ni nada dramático:

| Cambio | Deriva resultante |
|---|---|
| Esferas del pie de 5 mm → 10 mm | 3.5 cm/s |
| Desvío bilateral de 0.05 rad en tobillo | 7.9 cm/s |
| Desvío bilateral de 0.02 rad en cadera | 6.1 cm/s |

**Conclusión**: el gap puede explicarse solo por el contacto de los pies, o por
uno a tres grados de diferencia efectiva en la postura nominal.

### El dato que reordena todo

Codex midió en MuJoCo que **los límites de motor nunca se rozan**: la rodilla
llega a 45 N·m contra 139 disponibles; el tobillo a 20.8 contra 50. Con
`cmd=0.4` en régimen, ≤30.6 N·m y ≤5.6 rad/s.

Entonces, si en Isaac los topes oficiales tiran el robot, **el problema no es
que falte fuerza**: Isaac ya está en una trayectoria o un contacto incorrecto,
y los límites solo lo revelan. **El síntoma no era la causa.** (Nota:
`velocity_limit_sim` sí es una restricción dura — PhysX frena activamente la
articulación que lo excede, y NVIDIA advierte que valores ajustados pueden
perjudicar la convergencia.)

### Orden de ataque (de Codex)

1. **Contacto pie-piso**. Cuatro esferas de 5 mm son extremadamente sensibles.
   Leer los `contact_offset` / `rest_offset` **efectivos del USD**, no asumir
   los valores por defecto.
2. **Solver y contacto como conjunto**: probar 4/0, 8/4, 16/8 y 32/16 buscando
   *convergencia*, no simplemente "más iteraciones".
3. **Ceros articulares**: signos gruesos poco probables; desvíos pequeños muy
   probables.
4. **PD implícito**: descartado como causa primaria, pero no su interacción con
   los límites duros.

### El primer experimento: un A/B puro

Partiendo de la configuración buena (un solo grupo de actuadores,
`effort = 300`), cambiar **solo** `velocity_limit_sim` de 100 a los valores
oficiales, y registrar —**antes de la primera inclinación**— `q`, `dq`,
`q_des`, `tau = kp(q_des − q) − kd·dq`, y el impulso normal de cada esfera del
pie.

Cómo leer el resultado:

| Lo que se observe | Qué significa |
|---|---|
| `dq` cruza primero 20/32/37 | el frenado duro es el disparador; falta explicar por qué Isaac genera esas velocidades |
| El primer pico ocurre exactamente al apoyar el pie | es el contacto / el manifold |
| Nunca se acerca al límite y aun así cae | valor efectivo equivocado, mala asignación a articulaciones, o semántica numérica distinta |
| Solo cruza límites *después* de inclinarse | los límites son consecuencia, no causa |
| Con velocidad-sola queda de pie | repetir con fuerza-sola; si esa falla, comparar el límite de PhysX contra PD explícito con recorte por software |

Para cerrar definitivamente lo de los signos: robot con base fija, perturbación
de +0.01 rad articulación por articulación, y comparar el desplazamiento de las
ocho esferas de los pies contra MuJoCo.

---

## Otros pendientes conocidos

- **El detector confunde la mesa con el reloj**: los rangos de color son
  demasiado permisivos.
- **`buscar_persona` no gira**: espera pasivamente en vez de darse vuelta.
- **El planificador es de reglas**: la estructura para el modelo de lenguaje ya
  está; falta proveedor y credenciales.
- **Reloj simulado** (`/clock` + `use_sim_time`): sin él, los plazos medidos en
  tiempo de pared no significan nada con el simulador al 20 %. Ya nos rompió un
  timeout de navegación.
- **El tablero muere solo** con `ExternalShutdownException` y hay que revivirlo.
  Corre bajo supervisor, pero el supervisor también se cae. Sin diagnosticar.
- **Leer el reloj de verdad** con un modelo con visión, en vez de la hora fija.
- **Renombres pendientes**: la carpeta local y las rutas en la VM todavía dicen
  `go2`.

---

## La disciplina, ganada a los golpes

El 28-jul se perdió un día entero apilando "mejoras" sobre un sistema que
funcionaba, sin verificar entre una y otra. Cuando se rompió, no se sabía cuál
había sido, y encontrarlo llevó horas de bisección con git.

- **Una sola variable física por cambio**, nunca un paquete.
- Después de CADA cambio: **prueba de quedarse parado, y después de caminar**.
  Tres minutos por cambio son baratos comparados con otro día de bisección.
- Si una "mejora" no mejora una métrica declarada, **se revierte**.
- **"Oficial" no significa "compatible con esta policy"**: copiar de
  `unitree_rl_lab` el límite de los tobillos (25 N·m en vez de los 50 del URDF
  de entrenamiento) tiró el robot y costó una tarde.
- **Nunca cantar victoria con una sola lectura**: esperar a que el sistema se
  estabilice y mirar varias muestras.
- El banco de pruebas necesita el mismo rigor que el código: los peores
  desvíos del día vinieron de pruebas que leían logs viejos, contaban un robot
  congelado como "de pie", o se mataban a sí mismas.
