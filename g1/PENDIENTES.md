# Los dos problemas abiertos, y por dónde seguir

Estado al 30-jul. Punto de retorno seguro: etiqueta de git `known-good`
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

- **Auditoría para quitar las “rueditas de ayuda”**: se hará inmediatamente
  después de la próxima corrida integral aprobada visualmente. No será una
  eliminación a ciegas. Primero se registrará, para cada dato y decisión, de
  dónde sale y qué equivalente tendrá en el G1 físico. La primera lista que hay
  que confirmar en el código incluye:

  1. plan local de respaldo ante falla del modelo;
  2. coordenadas conocidas del reloj y cualquier posición interna leída de
     Isaac;
  3. detectores por color o forma ajustados sólo a esta habitación;
  4. profundidad perfecta o sin ruido usada como si fuera medición real;
  5. `attach_payload`, que prueba carga pero no agarre;
  6. respuestas simuladas, modos de prueba y valores fijos habilitables por
     variables de entorno;
  7. publicación directa de movimiento que pueda evitar
     `mobility_authority`;
  8. éxito declarado a partir de un valor pedido y no de una medición.

  El perfil normal deberá **fallar de forma visible y segura**: ante falta de
  modelo, sensor o dato válido, pasar a `STAND` y pedir ayuda o detenerse. Las
  inyecciones útiles se conservarán sólo bajo un perfil `test` explícito, con
  su origen mostrado en el tablero. El cierre será una misión bajo perfil de
  despliegue, seguida por fallas inducidas de red, visión y localización; ninguna
  deberá activar un reemplazo silencioso. La clasificación, el perfil sin
  ayudas y sus criterios están fijados en
  [`DEPLOYMENT_READINESS_PLAN.md`](DEPLOYMENT_READINESS_PLAN.md).

- **Recuperación visual desde otro lugar**: todavía no existe la capacidad
  `relocate_viewpoint`. Después de dos barridos el agente debe pedir ayuda; no
  puede renombrar `scan_for_table` y fingir que cambió de posición. Se agregará
  sobre Nav2 con mapa de obstáculos y posiciones de observación comprobables.

- **Demo objetivo actualizada**: la misión vigente ya no busca personas. Debe
  escuchar una orden, guardar `home`, encontrar y leer el reloj, elegir por la
  hora una mesa roja o azul no registrada, encontrarla, tomar su objeto y
  regresar al inicio. El alcance y la escalera de pruebas están fijados en
  [`DEMO_TARGET.md`](DEMO_TARGET.md).
- **Mapeo, ubicación y navegación como en el G1 real**: la demo actual usa
  coordenadas conocidas de Isaac. Es un escalón válido para probar locomoción,
  pero no la arquitectura final. Unitree declara cámara de profundidad y LiDAR
  3D en el G1 EDU; el modelo exacto, su driver y sus topics deberán confirmarse
  contra la unidad que se compre. NVIDIA permite que el LiDAR simulado publique
  mensajes ROS 2 estándar `PointCloud2` y `LaserScan`, por lo que el simulador
  puede reproducir la frontera real sin entregar posiciones perfectas.

  Camino de implementación y prueba, una frontera por vez:

  1. hacer que el LiDAR aislado entregue una nube no vacía, con frecuencia,
     alcance y referencias espaciales medidos;
  2. publicar la cadena completa de referencias espaciales
     `map → odom → base_link → lidar` y comprobarla en RViz;
  3. convertir el LiDAR 3D a la vista 2D que consuma la primera integración,
     sin leer geometría interna de Isaac;
  4. construir y guardar el mapa con `SLAM Toolbox`, la opción recomendada por
     Nav2 para este caso;
  5. volver a encender en otra pose y localizarse sobre el mapa guardado;
  6. integrar Nav2 con mapa global, mapa local de obstáculos, planificación de
     ruta y evasión de obstáculos;
  7. conectar Nav2 como solicitante de `mobility_authority` y pasar la salida
     autorizada por Collision Monitor, el último filtro de software antes de
     `/cmd_vel`; ninguna fuente publicará por fuera de esa cadena;
  8. reemplazar el navegador simple detrás de la misma Action
     `NavigateToPose`, sin cambiar al agente;
  9. agregar puntos de observación alcanzables para buscar objetos y la
     capacidad `relocate_viewpoint`;
  10. mostrar en el tablero la nube del LiDAR, el mapa, la ubicación estimada,
      la ruta, los obstáculos y la incertidumbre.

  Reparto de responsabilidades que no se debe mezclar: LiDAR mide paredes y
  obstáculos; SLAM arma el mapa y ubica al robot; Nav2 decide por dónde llegar a
  una coordenada; la cámara reconoce que algo es un reloj o una mesa; el agente
  decide cuál de esos lugares visitar; la policy de locomoción convierte la
  orden de velocidad en pasos estables.

  La primera meta no será “Nav2 completo”, sino una prueba discriminante: mapa
  hecho sólo con sensores, reinicio en otra pose, localización correcta y viaje
  al reloj sin usar ninguna coordenada interna del simulador. Después se agrega
  la búsqueda de mesas no registradas en posiciones de observación seguras.
- **Lectura visual en Azure AI Foundry, integrada**: el detector de la Jetson
  manda sólo el recorte JPEG al servidor externo; éste usa la API `v1` y una
  salida con formato estricto. `gpt-4.1-mini` leyó `09:00` 3/3 veces con red
  limpia (`1,19–2,72 s`) y 3/3 con `wifi-bad` (`1,83–2,89 s`). Con corte total,
  tres pedidos vencieron en `2 s` y el cuarto se rechazó localmente en `0 ms`.
  Con el robot recién navegado frente al reloj, el camino vivo
  cámara→detector→recorte→servidor→Azure leyó `09:00` 3/3 veces
  (`1,77–2,29 s` punta a punta).
  Las credenciales y el endpoint viven sólo en `.env` de la VM con permisos
  `600`; no están en Git. La misión ROS completa ya invoca este paso y publica
  por separado la imagen exacta, el texto literal del modelo y el dato
  validado. Falta la confirmación visual de Lucas sobre el tablero nuevo.
- **Carrera del lanzador visual, corregida**: `run_demo.sh clock` soltaba el
  robot, esperaba dos segundos y mandaba el objetivo. Una corrida tomó
  navegación y perdió la concesión 12 s después; la prueba que espera la
  confirmación real de `frozen` y `active` pasó con `10,4 cm`, `1,3°` y 4/4
  detecciones. El comando visual ahora usa ese mismo verificador. El árbitro
  también registra cualquier vencimiento que ocurra dentro de su reloj.
- **Regreso a `home` sin carga, validado numéricamente**: la prueba guarda la
  pose recibida por ROS al iniciar —no supone que el origen es `(0,0)`—, navega
  hasta el reloj y vuelve a esa pose. Tres repeticiones regresaron con errores
  de `9,1`, `9,5` y `7,9 cm`, y errores de orientación de `2,7°`, `1,3°` y
  `4,2°`, sin caída. Las idas quedaron a `11,2`, `9,6` y `8,5 cm` del destino.
  La primera captura tuvo un giro transitorio de `13,6°`; las otras dos fueron
  `-0,1°` y `-0,5°`, por lo que queda registrado pero no se declara tendencia.
  Falta la inspección visual de Lucas y usar esta pose dentro del ejecutor de
  la misión.
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
- **Barrido visual activo, validado**: el robot cubre 360° con cinco vistas,
  usa RT-DETR o color sólo para filtrar candidatos y confirma con Grounding
  DINO más profundidad. Dos misiones completas hasta ese punto encontraron la
  mesa roja en la cuarta vista con una llamada remota y terminaron en `STAND`.
  La primera base se desplazó 17,1 cm durante el barrido: es aceptable en sala
  abierta, no para manipular. El JPEG exacto de Grounding DINO, su pedido y su
  respuesta literal ya aparecen en el tablero.
- **Preaproximación visual, validada**: la primera versión a `0,9 m` llegó a
  quedar a `0,543 m` y perdió la mesa en otras vistas. Se separó la llegada
  gruesa del control fino, siguiendo el patrón del servidor de docking de
  Nav2. Con `2,2 m` desde la superficie detectada, la corrida integral llegó
  con `9,8 cm` de error, reconfirmó la mesa a `1,968 m`, confianza `0,94`,
  cuerpo a `0,737 m` y brazos listos con `0,0254 rad` de error.
- **Alineación visual fina, implementada y medida**: expone la Action estándar
  `DockRobot`, sigue mediciones locales nuevas y entrega siempre la movilidad
  a `STAND`. Desde lejos, roja terminó a `1,6 cm` y `1,58°`; azul se estancó
  una vez a `1,39 m` y el reintento idéntico terminó a `0,2 cm` y `0,25°`.
  Nav2 admite tres reintentos; esta adaptación permite sólo uno porque es lo
  medido. Falta la corrida integral y la confirmación visual de Lucas.
- **Objeto sobre la mesa, localizado en 3D**: el cilindro liso de la escena se
  reconoce como `cup`, no como `bottle`. Con el umbral general sin cambios,
  seis cuadros positivos dieron `0,49–0,62` y cinco controles sin objeto
  `0,03–0,04`. Tres posiciones publicadas en `Detection3DArray` quedaron a
  `3,2 cm` en horizontal y `5–6 mm` en altura de la referencia física, con
  cuerpo a `0,739 m` y dueño `STAND`. Es una superficie visible para búsqueda
  y aproximación, no la orientación completa que exige el agarre. La corrida
  integral encontró el objeto como `cup` con confianza `0,605`, a `28,8 cm`
  del punto visible de la mesa, y conservó honestamente la calidad
  `visible_surface_only`.
- **Transporte con carga separado del agarre**: `attach_payload` agrega en
  caliente una masa absoluta y verificable entre ambas muñecas y muestra un
  bulto sin masa adicional. La misión ya lo ejecutó después de
  `align_with_table`, adoptó la pose de transporte y regresó a `home`. La
  revisión visual posterior **rechazó** esa pose: el bulto aparece sobre la
  zona de la pelvis y los brazos no representan un transporte creíble. No
  valida contacto, dedos ni retención del objeto. La medición aprobó `0,5 kg`:
  tres caminatas frenaron en `0–3 cm`
  y tres navegaciones llegaron a `9–10 cm`. Con `1 kg` las piernas pasaron,
  pero el hombro derecho quedó a `1,9°` del objetivo contra `1,7°` permitidos;
  por eso la misión queda en `0,5 kg` y no se sube a `2 kg`.
- **Tablero entre misiones, corregido localmente**: el proceso conservaba la
  historia y los eventos del modelo aunque el agente se reiniciara. También
  mostraba la última orden de brazos en vez de la pose medida. La corrección
  separa por `mission_id`, limpia los datos al volver a `idle` y usa
  `/g1/arm_status`. Falta desplegarla y comprobarla en el navegador.
- **Planificador semántico y ejecución adaptable integrados**:
  `gpt-4.1-mini` recibe un catálogo explicado, propone el plan en JSON y el
  servidor y la Jetson lo validan independientemente. Después de cada paso
  puede continuar, cerrar con `complete`, reintentar o cambiar sólo lo
  pendiente. El tablero conserva la entrada exacta, la respuesta literal, la
  evidencia y el plan aceptado. La revisión final también recibe su evidencia;
  `complete` se rechaza si hubo una falla o aún quedan pasos.
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
