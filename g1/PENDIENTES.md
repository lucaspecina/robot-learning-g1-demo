# Los dos problemas abiertos, y por dónde seguir

Estado al 29-jul. Punto de retorno seguro: etiqueta de git `known-good`
(commit 86f8082) — de pie a 0.72 m, caminó 5.85 m, ciclo congelar/soltar
funcionando.

---

## Problema 1 — autoridad de `/cmd_vel`: estructura resuelta

**El síntoma**: cualquier nodo publica órdenes de velocidad cuando quiere. Si
dos publican a la vez, se anulan y el robot se queda quieto sin que ningún log
diga nada raro. Nos pasó con la navegación manteniendo la posición mientras una
prueba pedía caminar.

**Estado actual**: `mobility_authority.py` es el único publicador de
`/cmd_vel`. Navegación, espera, control manual y pruebas tienen entradas
separadas, piden una concesión exclusiva y dejan registro de cada transición.
`stand_hold.py` corrige la deriva al esperar; navegación se calla al terminar.
Se eliminó `/g1/hold`.

La integración pasó, en una misma corrida y reiniciando la pose antes de cada
movimiento: espera, caminata y navegación. Esto resuelve el conflicto de
autoridad, pero **no** la causa física de la deriva: el mantenimiento de pose
sigue siendo una capa necesaria y observable, no evidencia de que la física
esté bien.

Regresión activa del 29-jul, tres repeticiones:

- espera con `stand_hold`: aprobó dos veces; una tuvo un pico de 17 cm;
- caminata directa: no cayó, avanzó 2,47–2,56 m y frenó en 5–8 cm;
- trayectoria directa: 5,1°, 7,8° y 6,2° fuera de la recta.

Los 17 cm de espera alcanzan para sala abierta, no para manipular. Por eso el
problema no queda maquillado como “resuelto”: la precisión se debe medir y
cerrar en `stand_hold`, navegación y alineación, cada una por separado.

Experimento de una variable posterior: bajar solamente la corrección máxima de
`stand_hold` de `0,45` a `0,15 m/s`. Las tres repeticiones quedaron dentro de
`1`, `0` y `0 cm`; se conserva. La caminata posterior avanzó `2,56 m`, quedó a
`4,8°` de la recta y frenó en `5 cm`, sin caída.

El límite absoluto de `15 cm` laterales se retiró de la prueba básica: dependía
de cuánto avanzara el simulador durante el tiempo de pared y duplicaba el
criterio angular. La precisión absoluta se exige en navegación.

Navegación general también quedó medida. Bajar solamente la llegada de `35` a
`10 cm` funcionó una vez y luego se estancó a `14 cm`: a esa distancia pedía
apenas `0,04 m/s`, insuficiente para mover el bípedo. El segundo A/B agregó una
velocidad mínima de aproximación de `0,10 m/s`. Resultados: `7`, `8` y `9 cm`,
sin caída ni órbita. Se conservan ambos valores.

Primera prueba visual hacia el reloj, 29-jul: objetivo `(0,80, 1,80)` y
orientación `138,8°`; posición observada al terminar `(0,84, 1,73)` y
orientación `143,4°`. El error fue `7,5 cm` y `4,6°`. El recorrido visto desde
arriba fue coherente. Esto aprueba provisionalmente traslado y orientación,
pero no “mirar el reloj”: la escena y el detector fallaron visualmente.

Esto cierra el traslado general, no la manipulación. Junto a la mesa falta un
estado de alineación fina que mida mano, objeto y base y exija `2–3 cm`.

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

## Problema 2 — deriva de la policy anterior: cerrada al reemplazar el conjunto

La línea de investigación siguiente pertenece a `motion.pt`, la policy anterior
de Unitree. No continuar ajustando esa física: WBC-AGILE de NVIDIA la reemplazó
como base porque aporta juntos cuerpo, motores, retraso, frecuencias,
descriptor y policy para el G1 de 29 articulaciones.

La integración nueva se comparó directamente con NVIDIA:

- 29 articulaciones observadas y 12 controladas: mismos nombres y orden;
- 80 entradas, 12 salidas y 12 objetivos articulares: error máximo `0.0`;
- MuJoCo oficial: tres corridas idénticas, `1,82°` de desvío;
- Isaac oficial: `2,6°`, `6,3°` y `19,7°`;
- demo Isaac: `5,1°`, `7,8°` y `6,2°`.

Esto descarta la sospecha de entradas o ejes conectados en lugares equivocados.
También muestra que una orden frontal sin corrección no garantiza una línea
global, incluso en la escena oficial. La precisión de recorrido pertenece a
navegación; la precisión quieto pertenece a `stand_hold`.

### Historia de la policy anterior — conservar, no reabrir sin motivo

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

- **Demo objetivo actualizada**: la misión vigente ya no busca personas. Debe
  escuchar una orden, guardar `home`, encontrar y leer el reloj, elegir por la
  hora una mesa roja o azul no registrada, encontrarla, tomar su objeto y
  regresar al inicio. El alcance y la escalera de pruebas están fijados en
  [`DEMO_TARGET.md`](DEMO_TARGET.md).
- **Mapeo y localización como en el G1 real**: la demo actual usa coordenadas
  conocidas de Isaac. Es un escalón válido para probar locomoción, pero no la
  arquitectura final. El G1 EDU dispone de cámara de profundidad y LiDAR 3D.
  Hay que simular ambos sensores, construir el mapa a partir de sus mediciones
  y ubicar al robot dentro del mapa sin leer posiciones internas de Isaac.
  Flujo previsto:

  1. recorrer la habitación y guardar un mapa;
  2. en cada encendido, cargarlo y localizar al robot;
  3. usar el LiDAR para geometría y obstáculos;
  4. usar la cámara para reconocer reloj, color de mesa y objetos;
  5. guardar los objetos encontrados en un mapa con nombres.
  El LiDAR no identifica por sí solo que una forma es un reloj. Para una primera
  referencia compatible con Nav2, evaluar `SLAM Toolbox` y la conversión del
  LiDAR 3D a la representación necesaria; antes de implementarlo, confirmar
  los topics y drivers exactos entregados con la versión del G1 EDU comprada.
  Se hará después de cerrar visualmente locomoción, cámara y llegada al reloj.
- **Lectura visual en Azure AI Foundry, integrada**: el detector de la Jetson
  manda sólo el recorte JPEG al servidor externo; éste usa la API `v1` y una
  salida con formato estricto. `gpt-4.1-mini` leyó `09:00` 3/3 veces con red
  limpia (`1,19–2,72 s`) y 3/3 con `wifi-bad` (`1,83–2,89 s`). Con corte total,
  tres pedidos vencieron en `2 s` y el cuarto se rechazó localmente en `0 ms`.
  Con el robot recién navegado frente al reloj, el camino vivo
  cámara→detector→recorte→servidor→Azure leyó `09:00` 3/3 veces
  (`1,77–2,29 s` punta a punta).
  Las credenciales y el endpoint viven sólo en `.env` de la VM con permisos
  `600`; no están en Git. Falta validar visualmente el recorte nuevo e invocar
  este paso desde la misión ROS completa.
- **Carrera del lanzador visual, corregida**: `run_demo.sh clock` soltaba el
  robot, esperaba dos segundos y mandaba el objetivo. Una corrida tomó
  navegación y perdió la concesión 12 s después; la prueba que espera la
  confirmación real de `frozen` y `active` pasó con `10,4 cm`, `1,3°` y 4/4
  detecciones. El comando visual ahora usa ese mismo verificador. El árbitro
  también registra cualquier vencimiento que ocurra dentro de su reloj.
- **Teleoperación con Meta Quest 3 (después de cerrar locomoción e integración
  de AGILE)**: probar primero el flujo oficial de NVIDIA para G1. Isaac Lab
  2.3.x incorpora control del G1 desde dispositivos XR y conversión de los
  movimientos humanos a brazos/manos del robot. Evaluar por separado:
  conexión del Quest, brazos/manos con base fija, locomoción mediante los
  joysticks y grabación de demostraciones. No conectarlo a `/cmd_vel`
  directamente: deberá entrar como modo `MANUAL` mediante
  `mobility_authority`.
- **Reloj visual, pendiente de confirmación de Lucas**: el panel blanco de la
  primera prueba fue reemplazado por un display digital `09:00`, orientado
  hacia la pose de observación. La captura automática muestra las cuatro cifras
  completas y legibles. Falta que Lucas confirme la apariencia en Isaac y el
  tablero antes de cerrar el peldaño visual.
- **Detección temporal del reloj, numéricamente corregida**: la versión inicial
  etiquetó como reloj el `99,24 %` de una imagen blanca. Ahora combina color,
  área y forma. La prueba repetible obtuvo `0/3` falsos en el origen, `4/4`
  detecciones frente al display, centro `0,528` y cero confusiones con botella.
  Es válida para la escena controlada; en el robot real se reemplazará por un
  detector visual y profundidad, no por rangos de color.
- **Código viejo de personas**: `agent.py` todavía contiene la rama anterior.
  Hay que reemplazarla por selección de mesa roja/azul, búsqueda y regreso a
  `home`; no extender `buscar_persona`.
- **El planificador es de reglas**: la estructura para el modelo de lenguaje ya
  está; falta proveedor y credenciales.
- **Reloj simulado** (`/clock` + `use_sim_time`): sin él, los plazos medidos en
  tiempo de pared no significan nada con el simulador al 20 %. Ya nos rompió un
  timeout de navegación.
- **El tablero muere solo** con `ExternalShutdownException` y hay que revivirlo.
  Corre bajo supervisor, pero el supervisor también se cae. Sin diagnosticar.
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
