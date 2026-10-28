# Validación y análisis de datos de demostración antes de entrenar políticas de imitación

*Investigación del 2-oct-2026. Contexto: G1 con manos Brainco, 61 episodios de LeRobot v3.0 (LeRobot 0.4.1), una cámara de cabeza, estado y acción de 26 dimensiones, ACT con chunk 48.*

## Cómo leer este informe

- **"Fuente"** indica de dónde sale el chequeo. Si dice **adaptación propia**, ningún laboratorio lo publica así: es una técnica estándar que adapté a nuestros datos. Lo marco para cumplir la regla del proyecto de no hacer pasar adaptaciones por prácticas oficiales.
- Las citas de los papers se leyeron en las páginas HTML de arXiv. En algunos casos fue a través de un resumidor automático, así que una cita textual puede tener pequeñas diferencias con el PDF. Las marcadas *(aprox.)* conviene confirmarlas en el PDF antes de citarlas formalmente.
- Al final de la sección 0 hay **observaciones de una sonda rápida** que corrí sobre las grabaciones crudas (`~/Desktop/episodios/pick_mameluco`). Son lecturas preliminares, no resultados validados, y sirven para mostrar que los chequeos propuestos encuentran cosas reales en nuestros datos.

### Términos usados

| Término | Significado |
|---|---|
| **Episodio / demostración** | Una grabación completa de la tarea, de principio a fin. |
| **Cuadro (frame)** | Un instante grabado: imagen + estado + acción. Son 30 por segundo. |
| **Estado (`observation.state`)** | Posición *medida* de las articulaciones. |
| **Acción (`action`)** | Posición *ordenada* por la teleoperación. El robot la persigue con cierto retraso. |
| **Lag** | Cuántos cuadros tarda el estado en reflejar la acción. |
| **Held-out** | Episodios reservados que la política nunca ve al entrenar. Sirven para evaluarla. |
| **Fuga (leakage)** | Información del held-out que se filtró al entrenamiento y hace que la evaluación parezca mejor de lo que es. |
| **Idle** | Tramos en los que el operador no se mueve. |
| **Rollout** | Ejecutar la política en el robot (o en simulación) en lazo cerrado. |

---

## 0. Hallazgos principales

1. **Ningún laboratorio publica un "notebook de validación" estándar.** Lo que sí publican son reglas sueltas que se repiten:
   - entrenar la versión final sólo con episodios exitosos (RT-1, π0.5, DROID, GR00T);
   - recortar los tramos quietos (TRI LBM, Diffusion Policy, GR00T);
   - normalizar cada dimensión con cuantiles (π0.5 usa 1/99; TRI usa 2/98);
   - registrar timestamps reales y descartar saltos anómalos (documentación de despliegue de GR00T, ALOHA 2, UMI);
   - mirar cada episodio a mano (Unitree, GR00T).

   La lista práctica más concreta y pública es [`real_world_deployment.md` de Isaac-GR00T](https://github.com/NVIDIA/Isaac-GR00T/blob/main/getting_started/real_world_deployment.md).

2. **La curación no siempre ayuda.** TRI encontró que filtrar los tramos de poco movimiento *mejoró* las políticas de una sola tarea pero *empeoró* su modelo grande preentrenado ([arXiv 2507.05331](https://arxiv.org/html/2507.05331)). π0 sostiene que los datos de alta calidad solos "no enseñan a recuperarse de errores" ([arXiv 2410.24164](https://arxiv.org/html/2410.24164)). Para nuestro caso (una tarea, ACT), la evidencia apunta a filtrar fracasos y recortar los extremos quietos.

3. **La calidad del operador pesa mucho.** En robomimic, la misma tarea con demostraciones "Better" frente a "Worse" pasa de 22% a 58,7% de éxito con BC en Square ([arXiv 2108.03298](https://arxiv.org/abs/2108.03298)). Belkhale et al. muestran que la variabilidad en las acciones del operador perjudica, y que la diversidad útil es la del entorno, no la del operador ([arXiv 2306.02437](https://arxiv.org/abs/2306.02437)).

4. **El loss de validación no sirve para elegir checkpoints.** Según robomimic, "the best validation policy is 50 to 100% worse than the best performing policy" ([robomimic study](https://robomimic.github.io/study/)). El held-out sirve para diagnosticar (error por articulación, sesgos), no para decidir.

5. **Nuestro pipeline no tiene timestamps reales.**
   - `xr_teleoperate` guarda sólo `idx` por cuadro.
   - El conversor de Unitree no pasa `timestamp`, así que LeRobot escribe `frame_index/30`.
   - LeRobot v3 ya no tiene el chequeo `check_timestamps_sync` que existía en v0.3.

   Consecuencia: **ningún chequeo de timestamps puede detectar cuadros perdidos o jitter en nuestro dataset**. Hay que inferirlos de forma indirecta (cuadros de video repetidos, saltos en las articulaciones) y agregar timestamps reales en las próximas grabaciones.

6. **La herramienta oficial ya trae varios de estos chequeos.** El LeRobot Dataset Visualizer (Space de Hugging Face) incluye:
   - alineación temporal estado–acción por correlación cruzada;
   - autocorrelación de acciones con un tamaño de chunk sugerido;
   - variabilidad de velocidad entre demostradores;
   - banderas de episodios de "poco movimiento" y "bruscos";
   - réplica 3D con URDF que soporta el G1.

   Conviene usarlo como referencia oficial y replicar sus métricas en el notebook ([código](https://github.com/huggingface/lerobot-dataset-visualizer)).

7. **Métodos avanzados aplicables a ~60 demos:**
   - **DemInf** no necesita rollouts y los autores lo validan con 50 a 300 demostraciones.
   - **QoQ** necesita una política entrenada y unas pocas demos de validación.
   - **Demo-SCORE** y **CUPID** necesitan entre 25 y 100 rollouts etiquetados como éxito o fracaso.
   - **SCIZOR**, **DataMIL** y **Re-Mix** están pensados para datasets grandes o con muchas fuentes.

### Observaciones preliminares de la sonda sobre los datos crudos (65 episodios)

| Observación | Qué sugiere | Chequeo que la detecta |
|---|---|---|
| No faltan imágenes ni hay saltos en `idx` en ningún episodio. Las fechas de archivo tienen resolución de 1 s y no sirven como timestamps. | La integridad básica está bien, pero el tiempo real no es observable. | 1.1, 1.3 |
| La acción del brazo se refleja en el estado con **~3 cuadros de lag (~100 ms)**. El mejor lag por articulación va de 2 a 5 cuadros. | Retraso de control normal y bien dentro de un chunk de 48. | 3.1 |
| Las manos siguen la acción con mucho más lag (10–20 cuadros) y con un **desfase persistente**: error absoluto medio de 0,21 en `thumb_metacarpal` izquierdo. | La mano no llega a la posición ordenada, probablemente por contacto con la tela al agarrar (hipótesis). | 3.1, 3.2 |
| `left_thumb_metacarpal` llega a un máximo de **0,90 en `episode_0008`** y a **≤ 0,57 en todos los demás**. | Consistente con que 0008 sea el único episodio anterior al arreglo del pulgar. Es uno de los 4 que se excluyeron. | 6.1 |
| Los episodios `0015` (109 cuadros), `0028` y `0058` (85 cuadros) tienen las manos casi sin cerrar: máximo ≤ 0,15 en `thumb_metacarpal`. | Intentos sin agarre. La "apertura máxima de la mano" funciona como indicador de éxito. | 4.4 |
| El índice izquierdo (`left_index_proximal`) casi no se mueve: máximo ≈ 0,05–0,15 por episodio. | Articulación "casi congelada". Al normalizar por desvío, su ruido se amplifica. | 2.4 |
| La acción del `thumb_metacarpal` llega a **1,0** (el tope del rango), mientras que el estado máximo es 0,90. | Saturación de la orden en el límite. | 2.3 |
| Las estadísticas de normalización de la primera ACT coinciden con las de los **61** episodios, pero se entrenó con 54. | Los 7 episodios held-out contribuyeron a la normalización. Es una fuga menor (ver 8.4). | 8.4 |

---

## 1. Catálogo de chequeos

Formato de cada chequeo: **qué detecta**, **cómo se calcula** (numpy/pandas sobre `LeRobotDataset`), **gráfico**, **prioridad** para nuestro caso y **fuente**.

Notación de los fragmentos de código:
- `df` es el parquet de `data/` con las columnas `episode_index`, `frame_index`, `timestamp`, `observation.state` y `action`.
- `S` y `A` son arreglos `(T, 26)` de un episodio.
- `meta` es el parquet de `meta/episodes/`.

### 1. Integridad y estructura

**1.1 Esquema y conteos**
- **Detecta:** episodios vacíos, dimensiones o nombres equivocados, `total_frames` que no cuadra con la suma de los episodios, falta de cuantiles en `stats.json`.
- **Cómo:**
  - Leer `meta/info.json`.
  - Comprobar que `sum(meta.length) == info.total_frames == len(df)`.
  - Para cada episodio, comprobar que `frame_index` sea `0..length-1` sin huecos y que `dataset_to_index - dataset_from_index == length`.
  - Comprobar que `shape` coincida con 26, que `names` siga el orden `joint_names` del `data.json` y que `stats.json` tenga `q01` y `q99`. En nuestro dataset los tiene: LeRobot ≥ 0.4.0 calcula `q01, q10, q50, q90, q99`.
- **Gráfico:** tabla resumen. No hace falta gráfico.
- **Prioridad:** alta, porque es barato.
- **Fuente:** formato v3 en la [documentación de LeRobot](https://huggingface.co/docs/lerobot/lerobot-dataset-v3); `validate_frame` y `validate_episode_buffer` en `src/lerobot/datasets/` ([repo](https://github.com/huggingface/lerobot)).

**1.2 Valores inválidos**
- **Detecta:** NaN o Inf, y dimensiones constantes en todo el dataset.
- **Cómo:** `np.isfinite(S).all()` y `S.std(0) == 0`.
- **Prioridad:** alta.
- **Fuente:** la documentación de GR00T recomienda verificar `meta/stats` y que los rangos de acción "sean sensatos" cuando aparecen NaN o un MSE enorme ([finetune_new_embodiment.md](https://github.com/NVIDIA/Isaac-GR00T/blob/main/getting_started/finetune_new_embodiment.md)).

**1.3 Timestamps y cuadros perdidos**
- **Detecta:** cuadros perdidos o duplicados y jitter (variación irregular del intervalo entre cuadros).
- **Cómo:**
  - El chequeo clásico (`check_timestamps_sync` de LeRobot v0.3.3) es `np.abs(np.diff(ts) - 1/fps) <= tolerance_s`, ignorando los bordes entre episodios. La tolerancia por defecto es 1e-4 s.
  - **En nuestro caso es trivial y siempre pasa, porque los timestamps son sintéticos.** Hay que hacerlo igual, para documentar que no informa nada.
  - El sustituto indirecto es el chequeo 7.4 (cuadros de video repetidos) junto con el 2.1 (saltos articulares).
- **Gráfico:** histograma de `diff(ts)`.
- **Prioridad:** alta como acción para las *próximas* grabaciones: agregar `time.monotonic()` por cuadro en el `episode_writer` de `xr_teleoperate`. Esto modifica el flujo oficial de Unitree y hay que documentarlo.
- **Fuentes:**
  - LeRobot v0.3.3 `datasets/utils.py::check_timestamps_sync`, ausente en v0.4.x y en main.
  - GR00T: "Record full timestamps" y "Joint state sampling frequency should exceed camera FPS" ([real_world_deployment.md](https://github.com/NVIDIA/Isaac-GR00T/blob/main/getting_started/real_world_deployment.md)).
  - ALOHA 2 corta la sesión automáticamente si faltan datos y muestra la latencia al operador ([arXiv 2405.02292](https://arxiv.org/html/2405.02292)).

**1.4 Video frente a parquet**
- **Detecta:** desfase entre el número de cuadros del video AV1 y el de filas del parquet, y errores de decodificación.
- **Cómo:**
  - Para cada episodio, `(to_timestamp - from_timestamp) * fps ≈ length`.
  - Decodificar una muestra de índices al azar con `LeRobotDataset[i]`. Si el cuadro más cercano está a más de `tolerance_s`, LeRobot lanza `FrameTimestampError`.
- **Gráfico:** dispersión de `length` frente a cuadros de video.
- **Prioridad:** media.
- **Fuente:** `video_utils.py::decode_video_frames_torchcodec` y `_query_videos` en [LeRobot](https://github.com/huggingface/lerobot).

**1.5 Trazabilidad**
- **Detecta:** no saber de qué grabación cruda viene cada episodio de LeRobot. El conversor reindexa a 0..60.
- **Cómo:** construir una tabla `episode_index → episode_XXXX crudo` con la fecha y el orden de grabación. Ya existe en parte en `learning/act/split.json`.
- **Prioridad:** alta. Sin esto no se pueden estudiar la deriva (sección 6) ni el arreglo del pulgar.
- **Fuente:** adaptación propia.

### 2. Calidad de las señales articulares

**2.1 Saltos (discontinuidades)**
- **Detecta:** un salto físicamente imposible entre dos cuadros. Suele indicar un cuadro perdido, un glitch de la cinemática inversa (IK, el cálculo que convierte la pose de la mano del operador en ángulos de articulación) o el reinicio de la teleoperación.
- **Cómo:**
  - `dq = np.abs(np.diff(A, axis=0))`.
  - Marcar los cuadros en que `dq > v_max * dt * k`, con `v_max` el límite de velocidad del URDF del G1 por articulación, `dt = 1/30` y `k ≈ 1,5`.
  - Sin URDF, usar un umbral robusto: `dq > mediana + 8·MAD` por dimensión. MAD es la desviación absoluta mediana, una medida de dispersión que no se deja arrastrar por los extremos.
  - Hacerlo por separado para `S` y para `A`.
- **Gráfico:** heatmap de episodio × dimensión con el máximo `dq` normalizado. Además, la serie temporal del episodio peor con el salto marcado.
- **Prioridad:** alta.
- **Fuentes:** GR00T: "Detect and remove abnormal jumps in robot state sequences" ([real_world_deployment.md](https://github.com/NVIDIA/Isaac-GR00T/blob/main/getting_started/real_world_deployment.md)). UMI aplica "kinematics and dynamics feasibility filtering" ([arXiv 2402.10329](https://arxiv.org/html/2402.10329)).

**2.2 Ruido y jitter de alta frecuencia**
- **Detecta:** temblor de la mano del operador o ruido del seguimiento del visor XR.
- **Cómo:**
  - Por episodio y dimensión, `np.std(np.diff(A, n=2, axis=0))`, la segunda diferencia (aproxima la aceleración).
  - Alternativa: la energía del espectro por encima de ~5 Hz con `np.fft.rfft`.
  - Como versión "oficial", replicar el puntaje *jerky* del visualizador de LeRobot: media de |Δa| sobre las dimensiones activas, normalizada por el rango del motor.
- **Gráfico:** caja por episodio, y espectro promedio por articulación.
- **Prioridad:** media.
- **Fuentes:**
  - Visualizador de LeRobot: `fetch-data.ts`, "Per-episode jerkiness: mean |Δa| across dimensions active in that episode, normalized by motor range" ([código](https://github.com/huggingface/lerobot-dataset-visualizer)).
  - GR00T pide quitar trayectorias con "discontinuous actions".
  - Para una métrica de suavidad más formal, SPARC ([Balasubramanian et al. 2015](https://link.springer.com/article/10.1186/s12984-015-0090-9)). Prioridad baja.

**2.3 Saturación en límites**
- **Detecta:** órdenes que chocan contra el límite articular o contra el tope del rango de la mano (0..1). El robot no puede seguir esa orden y la política aprende a pedir algo inalcanzable.
- **Cómo:**
  - Fracción de cuadros con `A[:, j] >= hi_j - eps` o `<= lo_j + eps`, con `lo` y `hi` tomados del URDF para los brazos y de `[0, 1]` para las manos.
  - Comparar la saturación de `A` con la de `S`.
- **Gráfico:** barras de % de saturación por dimensión, e histograma por dimensión con los límites marcados.
- **Prioridad:** alta para las manos. En la sonda, la acción del pulgar llega a 1,0.
- **Fuente:** GR00T recomienda evitar operar cerca de los límites articulares ([real_world_deployment.md](https://github.com/NVIDIA/Isaac-GR00T/blob/main/getting_started/real_world_deployment.md)).

**2.4 Articulaciones congeladas o casi constantes**
- **Detecta:** dimensiones que no varían en general o dentro de un episodio. Pueden ser un sensor muerto, un dedo que no se usa o un problema de mapeo del retargeting (la traducción de la mano del operador a los dedos del robot).
- **Cómo:**
  - `S.std(0)` global y por episodio.
  - Rango `q99 - q01` por dimensión.
  - Marcar si el rango es menor a ~2% del rango físico.
  - **Ojo con la normalización media/desvío:** si una dimensión tiene un std diminuto, ε de ruido se convierte en muchos desvíos.
- **Gráfico:** heatmap de episodio × dimensión con el std.
- **Prioridad:** alta.
- **Fuente:** adaptación propia. El riesgo de normalización se ve en la elección de cuantiles en lugar de min-max por parte de π0.5 ([arXiv 2504.16054](https://arxiv.org/html/2504.16054)) y TRI ([arXiv 2507.05331](https://arxiv.org/html/2507.05331)).

**2.5 Distribución por dimensión**
- **Detecta:** multimodalidad (varios estilos de agarre), colas largas y la diferencia entre min-max y cuantiles.
- **Cómo:** histograma o violín por dimensión, con `mean ± std`, `q01/q99` y `min/max` superpuestos.
- **Prioridad:** media.
- **Fuentes:** π0.5 normaliza "using the 1% and 99% quantile of each action dimension" ([arXiv 2504.16054](https://arxiv.org/html/2504.16054)). TRI usa percentiles 2/98, recortados a [-1,5, 1,5] ([arXiv 2507.05331](https://arxiv.org/html/2507.05331)). LeRobot ofrece los modos `MEAN_STD`, `MIN_MAX` y `QUANTILES` (`configs/types.py`).

### 3. Coherencia acción–estado

**3.1 Lag de control**
- **Detecta:** el retraso entre la orden y el movimiento, y también una desincronización estado–acción en el registro.
- **Cómo:**
  - Por dimensión, correlación cruzada entre `Δa_t` y `Δs_{t+k}` para `k = 0..30`; el lag es el k del pico.
  - Alternativa: el `k` que minimiza `mean|a_t − s_{t+k}|`.
  - Agregar por episodio para detectar episodios con un lag anómalo (síntoma de desincronización).
- **Gráfico:** curva de correlación según el lag, por articulación, y boxplot del lag por episodio.
- **Prioridad:** alta.
- **Fuentes:**
  - Visualizador de LeRobot, sección "State–Action Temporal Alignment": "Per-dimension cross-correlation between Δaction(t) and Δstate(t+lag)… The peak lag reveals the effective control delay" (`action-insights-panel.tsx`, [código](https://github.com/huggingface/lerobot-dataset-visualizer)).
  - UMI mide las latencias de cámara, propiocepción (el sentido de la propia postura, aquí los ángulos medidos) y ejecución, y las compensa ([arXiv 2402.10329](https://arxiv.org/html/2402.10329)).
  - Diffusion Policy tolera "latency up to 4 steps" ([arXiv 2303.04137](https://arxiv.org/html/2303.04137)).

**3.2 Error de seguimiento**
- **Detecta:**
  - desfases sistemáticos, como una mano que no alcanza la orden porque aprieta la tela, o un problema de calibración;
  - acciones que son copia del estado, un bug de grabación que deja a la política sin nada que aprender.
- **Cómo:**
  - `e = A[:-k*] − S[k*:]` con el lag del 3.1.
  - Por episodio y dimensión: media (sesgo) y std.
  - Chequeo extra: `np.allclose(A[:-1], S[1:])` para descartar la copia.
- **Gráfico:** en las manos, el error de seguimiento superpuesto a la apertura, con las fases de agarre sombreadas.
- **Prioridad:** alta para las manos.
- **Fuente:** adaptación propia. Unitree propone validar reproduciendo las acciones (`eval_robot/replay_robot.py`) y evaluando offline sobre el dataset (`eval_g1_dataset.py`) ([unitree_IL_lerobot](https://github.com/unitreerobotics/unitree_IL_lerobot)).

**3.3 Autocorrelación de la acción y tamaño de chunk**
- **Detecta:** la escala temporal en que la acción "se vuelve otra". Sirve como referencia para `chunk_size` y `n_action_steps`.
- **Cómo:** autocorrelación de cada dimensión de `A` por episodio; el lag donde cae por debajo de 0,5; la mediana entre dimensiones.
- **Gráfico:** curvas de autocorrelación según el lag.
- **Prioridad:** media. Es informativo; no reemplaza las ablaciones de chunk.
- **Fuente:** visualizador de LeRobot, "Action Autocorrelation… Where autocorrelation drops below 0.5… natural action chunk boundary" (`action-insights-panel.tsx`). Es una heurística de la herramienta, no de un paper.

### 4. Calidad de cada demostración

**4.1 Duración**
- **Detecta:** episodios abortados (cortos) o con titubeos y reintentos (largos).
- **Cómo:** `meta.length`; outliers por `|x − mediana| / MAD > 3` o por IQR (rango intercuartílico).
- **Gráfico:** histograma, más la duración frente al orden de grabación.
- **Prioridad:** alta.
- **Fuentes:**
  - π0.5 entrena su post-training con episodios "filtered down to successful episodes that are below a fixed length threshold" ([arXiv 2504.16054](https://arxiv.org/html/2504.16054)).
  - ALOHA Unleashed: "shorter episodes collected by operators tend to have less mistakes" ([arXiv 2410.13126](https://arxiv.org/html/2410.13126)).
  - El visualizador de LeRobot trae un filtro de outliers por longitud.

**4.2 Tramos quietos al principio y al final**
- **Detecta:** los cuadros en que el operador todavía no arrancó, o ya terminó y sigue grabando.
- **Cómo:**
  - TRI define un umbral de movimiento respecto de la pose inicial (5 cm o 15° de la pinza) y descarta todo lo anterior.
  - Equivalente articular (**adaptación**): el primer cuadro con `||S_t[:14] − S_0[:14]||_∞ > θ`, con θ ≈ 0,05 rad, y lo mismo hacia atrás desde el final.
  - Reportar los segundos recortables por episodio.
- **Gráfico:** barras apiladas por episodio (quieto inicial / activo / quieto final).
- **Prioridad:** alta.
- **Fuentes:** TRI LBM ([arXiv 2507.05331](https://arxiv.org/html/2507.05331)); GR00T: "Head-tail trimming: remove idle segments" ([real_world_deployment.md](https://github.com/NVIDIA/Isaac-GR00T/blob/main/getting_started/real_world_deployment.md)).
- **Precaución:** TRI vio que este filtrado *empeoró* su modelo preentrenado. Es otro motivo para medir antes y después.

**4.3 Pausas a mitad de la demostración**
- **Detecta:** pausas a mitad del episodio. Con políticas de un solo paso hacen que la política "se trabe".
- **Cómo:** segmentos de ≥ N cuadros (por ejemplo 15, medio segundo) con `||Δa||_∞ < ε`; por episodio, número y fracción del tiempo.
- **Gráfico:** mapa de raster con episodios en filas y tiempo en columnas, coloreado por idle.
- **Prioridad:** media. ACT con chunks es más robusto a esto.
- **Fuentes:** Diffusion Policy: "Idle actions occur when a demonstration is paused… BC-RNN and IBC often get stuck… when the idle actions are not explicitly removed", y llama a eliminarlas "common practice" ([arXiv 2303.04137](https://arxiv.org/html/2303.04137)). ACT motiva el chunking con las "pauses in the middle of a demonstration" ([arXiv 2304.13705](https://arxiv.org/html/2304.13705)).

**4.4 Éxito o fracaso**
- **Detecta:** demostraciones fallidas que se colaron en el dataset.
- **Cómo:**
  1. **Etiqueta manual**: columna `success` en una tabla por episodio. Lo hacen DROID, RT-1 y π0.5.
  2. **Indicadores automáticos** (adaptación): apertura máxima de los dedos (que haya cierre), la pose final de los brazos (distancia a la pose final mediana) y si existe una fase de cierre sostenido de N cuadros.
  3. Inspección visual del último cuadro de cada episodio en una grilla.
- **Gráfico:** grilla de últimos cuadros, y dispersión de duración frente a cierre máximo.
- **Prioridad:** alta.
- **Fuentes:** RT-1 usa episodios "all of which are successful" ([arXiv 2212.06817](https://arxiv.org/html/2212.06817)); DROID: "The data collector also marks whether the collected sequence was a success" ([arXiv 2403.12945](https://arxiv.org/html/2403.12945)); GR00T pide quitar "failed task executions"; Unitree recomienda borrar los episodios fallidos con su `data_editor`.

**4.5 Titubeos, reintentos y caminos redundantes**
- **Detecta:** reagarres, idas y vueltas, y trayectorias más largas que lo necesario.
- **Cómo** (adaptación):
  - Número de ciclos abrir-cerrar de la mano: cruces de un umbral con histéresis.
  - Longitud de camino articular `Σ||Δs||` dividida por la del episodio mediano.
  - Cambios de signo de la velocidad del codo.
- **Gráfico:** dispersión de longitud de camino frente a duración.
- **Prioridad:** media.
- **Fuente:** GR00T pide quitar "low-quality trajectories (e.g., redundant paths, discontinuous actions)".

**4.6 Consistencia entre demostraciones (velocidad y estilo)**
- **Detecta:** diferencias de ritmo o de estrategia entre episodios. Por ejemplo, qué mano entra primero o desde dónde se toma la prenda.
- **Cómo:**
  - **Velocidad:** media de `||Δa||` por episodio y su coeficiente de variación (CV) entre episodios. El visualizador de LeRobot llama "Consistent" a un CV < 0,2 y "large" a uno ≥ 0,4.
  - **Estilo:** remuestrear cada episodio a 100 puntos y calcular la distancia a la trayectoria medoide (la demostración más "central"). Alternativa: DTW, una alineación temporal que permite comparar demostraciones de distinta velocidad. Después, clustering para ver estrategias.
- **Gráfico:**
  - superposición de trayectorias normalizadas en el tiempo, por articulación, con la mediana en negro;
  - el heatmap "Cross-Episode Action Variance" del visualizador.
- **Prioridad:** alta.
- **Fuentes:**
  - Belkhale et al.: la variación en las acciones del experto "can increase action divergence, and thus high state diversity is not universally desirable" ([arXiv 2306.02437](https://arxiv.org/abs/2306.02437)).
  - LeRobot: "maintain consistent grasping behavior" ([il_robots](https://huggingface.co/docs/lerobot/il_robots)).
  - Los umbrales de CV son una heurística de la herramienta (`action-insights-panel.tsx`), no de un paper.

### 5. Calidad visual

**5.1 Desenfoque**
- **Detecta:** desenfoque por movimiento de la cabeza o por foco.
- **Cómo:** en gris, `cv2.Laplacian(img, cv2.CV_64F).var()` por cuadro (varianza del Laplaciano). Comparar contra la distribución de todo el dataset, ya que el umbral absoluto depende de la escena.
- **Gráfico:** serie por episodio, y grilla de los cuadros más borrosos.
- **Prioridad:** media.
- **Fuentes:** GR00T pide quitar "blurry frames". La medida de foco por Laplaciano viene de [Pech-Pacheco et al., ICPR 2000](https://dblp.org/rec/conf/icpr/Pech-PachecoCCF00.html); es técnica clásica de visión, no de robótica.

**5.2 Exposición e iluminación**
- **Detecta:** sobreexposición o subexposición, y cambios de iluminación entre sesiones.
- **Cómo:** luminancia media por cuadro, y % de píxeles mayores a 250 o menores a 5.
- **Gráfico:** luminancia media por episodio según el orden de grabación.
- **Prioridad:** media.
- **Fuente:** el blog de Hugging Face sobre datasets recomienda iluminación estable ([lerobot-datasets](https://huggingface.co/blog/lerobot-datasets#what-makes-a-good-dataset)). Lo leí a través de un resumen; ver "No verificado".

**5.3 Visibilidad de la tarea**
- **Detecta:** que la prenda o las manos queden fuera del encuadre de la cámara de cabeza.
- **Cómo:** inspección visual con una grilla de cuadros cada 1 s por episodio. Opcionalmente, un detector abierto ya disponible en el repo (`g1/perception`) para contar cuadros con manos visibles.
- **Prioridad:** media.
- **Fuente:** LeRobot: "you should be able to do the task yourself by only looking at the camera images" ([il_robots](https://huggingface.co/docs/lerobot/il_robots)).

**5.4 Cuadros de video congelados o repetidos**
- **Detecta:** la cámara que se trabó y repitió una imagen. **Es nuestro único detector indirecto de cuadros perdidos**, porque no hay timestamps reales (ver 1.3).
- **Cómo:** `np.abs(f_t − f_{t−1}).mean()` sobre las imágenes decodificadas; marcar corridas de ≥ 2 cuadros con diferencia ≈ 0 mientras `Δs` es distinto de 0. Sólo es sospechoso si el estado se mueve y la imagen no.
- **Gráfico:** serie de la diferencia entre cuadros contra la velocidad articular.
- **Prioridad:** alta.
- **Fuente:** adaptación propia. GR00T pide filtrar "Excessive latency in a single camera stream".

**5.5 Sincronía cámara–estado**
- **Detecta:** un corrimiento sistemático entre la imagen y el estado.
- **Cómo** (adaptación):
  - Correlación cruzada entre la energía de movimiento de la imagen (media de `|f_t − f_{t−1}|`, idealmente sobre la región de las manos) y la velocidad articular `||Δs_t||`.
  - El pico debería estar en un lag cercano a 0.
- **Gráfico:** curva de correlación según el lag.
- **Prioridad:** media.
- **Fuentes:** UMI mide la latencia de la cámara filmando un QR con el reloj del sistema ([arXiv 2402.10329](https://arxiv.org/html/2402.10329)). Es el patrón oro y requiere una sesión de grabación dedicada. GR00T recomienda reproducir el episodio con visualización sincronizada para detectar "poor image–action sync".

**5.6 Artefactos de compresión AV1**
- **Detecta:** pérdida de detalle al codificar el video.
- **Cómo:** sobre una muestra, PSNR o SSIM (medidas de fidelidad de imagen) entre el JPEG crudo y el cuadro decodificado.
- **Prioridad:** baja.
- **Fuente:** el docstring de `lerobot-dataset-viz` advierte que la imagen tiene artefactos de compresión del mp4.

### 6. Deriva entre sesiones y orden de grabación

**6.1 Estadísticas por episodio según el orden de grabación**
- **Detecta:** cambios de régimen, como el arreglo del pulgar, una recalibración, la fatiga del operador o un cambio de iluminación.
- **Cómo:**
  - Usar las estadísticas por episodio que LeRobot v3 ya guarda en `meta/episodes` (columnas `stats/{feature}/{stat}`) o recalcularlas.
  - Para cada dimensión, graficar media, q01 y q99 contra el orden de grabación.
  - Detectar puntos de cambio (change-points) con un CUSUM simple (suma acumulada de desvíos respecto de la media) o comparando ventanas antes y después.
- **Gráfico:** gráfico de control con una línea por articulación y bandas de mediana ± 3·MAD. Es el gráfico que muestra el salto del pulgar en `episode_0008`.
- **Prioridad:** alta.
- **Fuentes:** adaptación propia sobre metadatos oficiales (`stats/...` por episodio en `dataset_metadata.py`). La sensibilidad de la normalización a mezclar fuentes la muestra TRI, que reconoce un bug en el que "some datagrams were normalized incorrectly (with normalization parameters belonging to a different data source)" ([arXiv 2507.05331](https://arxiv.org/html/2507.05331)).

**6.2 Impacto en la normalización**
- **Detecta:** cuánto mueve un subconjunto sospechoso (por ejemplo, los episodios previos al arreglo) la media y el desvío usados para normalizar.
- **Cómo:** ya existe `toolkit/checks/normalization.py`. Extenderlo a "estadísticas con y sin el episodio X" (dejar uno afuera) y reportar el desplazamiento en desvíos.
- **Prioridad:** media.
- **Fuente:** adaptación propia.

**6.3 Variación entre operadores**
- **Detecta:** diferencias de estilo o de calidad entre operadores.
- **Cómo:** si hubo más de un operador, etiquetar los episodios y repetir los chequeos 4.6 y 6.1 por operador.
- **Prioridad:** depende de si hubo varios operadores.
- **Fuentes:** robomimic MH ([arXiv 2108.03298](https://arxiv.org/abs/2108.03298)). ALOHA Unleashed revisa "5–10 test episodes" de cada operador nuevo antes de habilitarlo ([arXiv 2410.13126](https://arxiv.org/html/2410.13126)).

### 7. Diversidad y cobertura

**7.1 Condiciones iniciales**
- **Detecta:** si la pose del robot y la posición de la prenda al inicio varían o son siempre iguales. Si son siempre iguales, la política no va a generalizar.
- **Cómo:**
  - Dispersión de PCA (análisis de componentes principales, para ver 26 dimensiones en un plano) sobre `S_0` de cada episodio.
  - Grilla de primeros cuadros.
  - Distancia entre embeddings de imagen de los primeros cuadros: un embedding es el vector que resume una imagen, por ejemplo la ResNet18 que ya usa ACT, o DINOv2.
- **Gráfico:** PCA coloreado por orden de grabación o por split; grilla de imágenes.
- **Prioridad:** alta.
- **Fuentes:**
  - Data Scaling Laws: "increasing the diversity of environments and objects is far more effective than increasing the absolute number of demonstrations" ([arXiv 2410.18647](https://arxiv.org/abs/2410.18647)).
  - RT-1: "data diversity is more essential than data quantity" ([arXiv 2212.06817](https://arxiv.org/html/2212.06817)).
  - LeRobot sugiere "10 episodes per location".

**7.2 Cobertura del espacio de estados**
- **Detecta:** zonas del espacio de estados visitadas por pocos episodios, que serán frágiles.
- **Cómo:** PCA o UMAP de todos los `S` (con submuestreo) coloreado por episodio; densidad de kNN (k vecinos más cercanos) por cuadro; fracción de cuadros "raros".
- **Gráfico:** proyección 2D con las trayectorias como líneas.
- **Prioridad:** media.
- **Fuente:** adaptación propia. El concepto de "state diversity" viene de Belkhale et al.

**7.3 Casi duplicados**
- **Detecta:** episodios casi idénticos que no aportan información nueva.
- **Cómo:** distancia DTW entre trayectorias, y similitud coseno de embeddings de imagen.
- **Prioridad:** baja con 61 episodios, pero se reutiliza en 8.2.
- **Fuentes:** SCIZOR deduplica tramos con embeddings de video y pose, eliminando los que superan 0,99 de similitud coseno dentro de un cluster ([arXiv 2505.22626](https://arxiv.org/abs/2505.22626)). DINOv2 hace lo mismo con imágenes ([arXiv 2304.07193](https://arxiv.org/html/2304.07193)).

### 8. Split train/held-out y fuga de información

**8.1 Separar siempre por episodio, nunca por cuadro**
- **Detecta:** el error de mezclar cuadros consecutivos del mismo episodio en train y held-out. Esos cuadros son casi idénticos, así que la evaluación queda inflada.
- **Cómo:** verificar que `set(train_eps) ∩ set(heldout_eps) = ∅` a nivel de episodio.
- **Prioridad:** alta.
- **Fuentes:** ACT (`utils.py::load_data`, 80/20 por episodio y al azar), Diffusion Policy (`sampler.py::get_val_mask`, por episodio con semilla), LeRobot main (`dataset.eval_split` retiene "the last ceil(n_episodes * eval_split) episodes per task"), y `lerobot-edit-dataset --operation.type split`, que reparte por fracciones **contiguas, sin mezclar**.

**8.2 Representatividad del held-out**
- **Detecta:** un held-out que no se parece al train (o que se parece demasiado).
- **Cómo:**
  - Comparar las distribuciones train/held-out por dimensión: test de Kolmogórov-Smirnov (KS), que mide cuánto difieren dos distribuciones, o la distancia de Wasserstein.
  - Distancia de cada cuadro held-out a su vecino más cercano en train, comparada con la misma distancia dentro del train (dejando afuera su propio episodio).
  - Si el held-out queda muy cerca, la evaluación es "en distribución". Si queda lejos, mide generalización.
- **Gráfico:** dos CDF (distribuciones acumuladas) de distancia al vecino más cercano; PCA con train en gris y held-out en color.
- **Prioridad:** alta.
- **Fuente:** adaptación propia. La eliminación de casi duplicados entre train y test es práctica de DINOv2.

**8.3 Estratificación y elección del split**
- **Nuestra situación:** el split actual (índices 0, 10, …, 60) intercala el held-out a lo largo de toda la grabación. Mide aprendizaje dentro de la misma distribución, pero no sensibilidad a la deriva.
- **Propuesta** (adaptación):
  1. Mantener el split intercalado como "en distribución".
  2. Agregar un split de "bloque temporal": los últimos N episodios, que es lo que hacen `eval_split` de LeRobot y las fracciones contiguas de `lerobot-edit-dataset`.
  3. Estratificar por duración y por estrategia (los clusters de 4.6).
  4. Con 7 episodios held-out, la varianza de la métrica es grande. Conviene validación cruzada por episodios (k-fold de 5) para el error offline.
- **Gráfico:** tabla de split con duración, estrategia y orden de grabación; barras del error offline por fold.
- **Prioridad:** alta.

**8.4 Fuga a través de la normalización**
- **Detecta:** estadísticas de normalización calculadas con los episodios held-out incluidos.
- **Cómo:** recalcular media y desvío sólo con train y comparar contra las guardadas en la política (`toolkit/checks/normalization.py --exclude-episodes ...`).
- **En nuestro caso:** las estadísticas de la primera ACT coinciden con las de los 61 episodios, así que el held-out entró a la normalización. El impacto probablemente sea chico, pero hay que medirlo y documentarlo.
- **Prioridad:** media.
- **Fuente:** adaptación propia. En LeRobot v0.4.x, `stats.json` se calcula sobre todo el dataset y no existe un split de validación (verificado en v0.4.4).

**8.5 Qué medir sobre el held-out**
- **Qué:** usar el held-out para diagnosticar con error por articulación, por fase de la tarea y por horizonte del chunk. **No** usarlo para elegir checkpoints.
- **Prioridad:** alta.
- **Fuentes:**
  - robomimic: "the best validation policy is 50 to 100% worse than the best performing policy" ([robomimic study](https://robomimic.github.io/study/)). El paper habla de una caída de "10% to 100%" (aprox.); los dos números vienen de páginas distintas del mismo grupo.
  - README de ACT: "Success rate and smoothness can improve way after loss plateaus" ([act](https://github.com/tonyzhaozh/act)).
  - Diffusion Policy usa `val_ratio: 0.00` en su configuración de robot real y elige checkpoints por `train_loss` ([diffusion_policy](https://github.com/real-stanford/diffusion_policy)).

---

## 2. Métodos avanzados de curación: lo que reportan los papers

| Método | Puntaje | Requisitos | Resultado reportado | ¿Sirve con ~60 demos? |
|---|---|---|---|---|
| **DemInf** (Hejna et al., Stanford + Google DeepMind, 2025) [arXiv 2502.08623](https://arxiv.org/abs/2502.08623) | Información mutua entre estado y chunk de acción, estimada con kNN (KSG) en latentes de VAEs. El puntaje por demo es el promedio temporal de las contribuciones. Premia demos "diversas pero predecibles". | Sin rollouts, sin política, sin etiquetas. **Asume que todas las demos son exitosas.** | "5-10% improvement in RoboMimic", mejoras en ALOHA y Franka reales. Los estimadores son "more stable with datasets of 50-300 demonstrations". | **Sí**, es el más práctico. Riesgo declarado por los autores: prefiere datos "predecibles" aunque no avancen, como las pausas. Hay que correrlo *después* de filtrar fracasos y recortar los tramos quietos. |
| **QoQ** (Lee et al., KAIST/Berkeley/NYU, ICRA 2026) [arXiv 2603.09056](https://arxiv.org/abs/2603.09056) | Funciones de influencia: cuánto reduce cada muestra el loss sobre demos de validación. Toma el máximo sobre las muestras de validación y promedia dentro de cada trayectoria. | Una política BC entrenada y unas pocas demos de validación buenas. Sin etiquetas de éxito. | Supera a otros selectores; números concretos *(aprox.)*: RoboMimic Can 99,2%; agarre de banana real 86,7 vs 56,7. | Sí, y es más barato que CUPID. |
| **CUPID** (Agia et al., Stanford, 2025) [arXiv 2506.19121](https://arxiv.org/abs/2506.19121) | "Influencia en el desempeño": influencia del loss BC (aproximada con TRAK) combinada con los retornos de rollouts de éxito o fracaso (REINFORCE). | Política entrenada + **25–100 rollouts etiquetados** + gradientes por demo. | "training with less than 33% of curated data can yield state-of-the-art diffusion policies" en RoboMimic. En Franka real *(aprox.)*: Bookshelf 84% vs 44% al quitar demos con correlaciones espurias. | Posible pero costoso: cada ronda necesita rollouts en el robot. |
| **Demo-SCORE** (Chen, Lessing, Liu, Finn, Stanford, 2025) [arXiv 2503.03707](https://arxiv.org/abs/2503.03707) | Un clasificador MLP diminuto, entrenado con estados de rollouts exitosos frente a fallidos, puntúa cada demo por su probabilidad media de éxito. | Política + rollouts etiquetados (100 por checkpoint en simulación; *aprox.* funciona con ~10). | Mejora absoluta de "15-35%" en simulación; ~+25–30% en ALOHA real *(aprox.)*. | Plausible con unos 40–50 rollouts. Ya está mostrado en ALOHA bimanual. Puede reducir la cobertura. |
| **SCIZOR** (Zhang et al., UT Austin + NVIDIA, 2025) [arXiv 2505.22626](https://arxiv.org/abs/2505.22626) | Por tramos de 2 s: predictor auto-supervisado de progreso (DINOv2) más deduplicación con embeddings. | Sin etiquetas ni rollouts. | +15,4% en promedio; +32,9 en Sirius-Fleet. | Pensado para datasets grandes; supone progreso lineal y sin pausas. |
| **DataMIL** (Dass et al., 2025) [arXiv 2505.09603](https://arxiv.org/abs/2505.09603) | Datamodels: estima qué clusters de un *pool grande* (por ejemplo OXE) mejoran la tarea objetivo. | Pool externo grande; mucho cómputo. | 61,0% vs 40,0% en 4 tareas reales. | No: selecciona datos de terceros, no limpia los nuestros. Sirve más adelante para co-entrenamiento. |
| **Re-Mix** (Hejna et al., 2024) [arXiv 2408.14037](https://arxiv.org/abs/2408.14037) | Pesos por dominio con DRO (optimización robusta a la peor distribución). | Muchos dominios. | +38% sobre pesos uniformes en RT-X. | No aplica a una sola tarea. |
| **Data Quality in IL** (Belkhale, Cui, Sadigh, NeurIPS 2023) [arXiv 2306.02437](https://arxiv.org/abs/2306.02437) | Marco conceptual: *action divergence* y *transition diversity*. | — | El ruido en las acciones del experto da un rendimiento "almost universally lower" a igual cobertura. | Guía para grabar: operador consistente, variedad en el entorno. |
| **robomimic** (Mandlekar et al., 2021) [arXiv 2108.03298](https://arxiv.org/abs/2108.03298) | Comparación por calidad de operador. | — | BC en Square: Worse 22,0%, Okay 27,3%, Better 58,7%. BC-RNN casi cierra la brecha en Can pero no en Square. | Aplica directamente: separar por operador o por sesión. |
| **AWE** (Shi, Sharma, Zhao, Finn, 2023) [arXiv 2307.14326](https://arxiv.org/abs/2307.14326) | Reetiqueta las demos como waypoints; no filtra. | — | Hasta +25% en simulación y +4–28% en ALOHA real. | Alternativa a recortar: simplifica las trayectorias. |

**Lo que dicen los laboratorios sobre cuánto importa la curación:**

- **TRI (LBM):** filtrar los tramos de poco movimiento ayudó a las políticas de una tarea, pero "led to a surprising decrease in performance for pretrained LBM". Por eso preentrenan sin filtrar y hacen fine-tuning con datos filtrados ([arXiv 2507.05331](https://arxiv.org/html/2507.05331)).
- **Physical Intelligence:**
  - π0: "training only on high-quality data does not teach the model how to recover from mistakes"; la combinación de datos de alta y baja calidad da el mejor comportamiento ([arXiv 2410.24164](https://arxiv.org/html/2410.24164)).
  - π0.5: post-training sólo con episodios exitosos y por debajo de un largo máximo ([arXiv 2504.16054](https://arxiv.org/html/2504.16054)).
  - π\*0.6 / RECAP: usa también los fracasos etiquetados, condicionando la política en la ventaja (cuánto mejor que lo esperado fue cada acción). Reporta que "more than doubles task throughput and roughly halves the task failure rate" ([arXiv 2511.14759](https://arxiv.org/html/2511.14759)).
- **Google DeepMind:**
  - Gemini Robotics hace "curate between 2000 and 5000 episodes of high-quality demonstration data" por tarea especializada ([arXiv 2503.20020](https://arxiv.org/html/2503.20020)).
  - ALOHA Unleashed observa que "some amount of suboptimal data may also be necessary, since this data contains recovery and retry behaviors" ([arXiv 2410.13126](https://arxiv.org/html/2410.13126)).
- **NVIDIA GR00T N1:** "we filter out low-quality trajectories", sin publicar el criterio ([arXiv 2503.14734](https://arxiv.org/html/2503.14734)). Su documentación de despliegue pide ≥ 100 episodios válidos (unos 30 para tareas muy acotadas) y "200+" para que sea más estable.
- **Octo:** excluyó datasets "too repetitive, have a low image resolution, or consist of excessively niche tasks" y dio más peso a los diversos ([arXiv 2405.12213](https://arxiv.org/html/2405.12213)).

**Recomendación para nuestro caso** (síntesis propia):
1. Primero, chequeos heurísticos y visuales (secciones 1–7), y filtrar los fracasos.
2. Después, como experimento con una sola variable: ACT con y sin los episodios marcados, con una métrica declarada antes (éxito en rollouts) y repetida.
3. Recién entonces DemInf o QoQ como puntaje automático, para contrastarlo con la revisión visual.
4. Demo-SCORE o CUPID sólo cuando haya un protocolo de rollouts etiquetados en el robot.

---

## 3. Herramientas reales

- **LeRobot Dataset Visualizer** (Space oficial, [huggingface.co/spaces/lerobot/visualize_dataset](https://huggingface.co/spaces/lerobot/visualize_dataset); [código](https://github.com/huggingface/lerobot-dataset-visualizer); [docs](https://huggingface.co/docs/lerobot/using_dataset_tools)).
  - Muestra video sincronizado con los gráficos de estado y acción, más paneles de Overview y Statistics (histograma de longitudes).
  - El panel **Action Insights** incluye autocorrelación, alineación estado–acción, varianza de velocidad y un heatmap de varianza entre episodios.
  - El panel **Filtering** marca episodios de poco movimiento y bruscos y outliers de longitud. Además **exporta el comando `lerobot-edit-dataset ... delete_episodes`** listo para usar.
  - Tiene réplica 3D con URDF, con soporte del G1 (`g1_body29_hand14.urdf`).
  - Funciona con datasets v3.0. Para uno local hay que servirlo o subirlo al Hub como privado; no lo probé con un dataset local.
- **`lerobot-dataset-viz`** (Rerun): un episodio en línea de tiempo, con imágenes y escalares de estado y acción; `--save 1` genera un `.rrd`. Rerun abre directamente datasets v2.1 y v3.0 (`rerun path/al/dataset`) ([docs de Rerun](https://rerun.io/docs/howto/logging-and-ingestion/lerobot)).
- **`lerobot-edit-dataset`** (v0.4.x): `delete_episodes`, `split` (por fracciones contiguas o listas explícitas de episodios), `merge`, `remove_feature`, `modify_tasks`. En main se suman `recompute_stats` y `reencode_videos`.
- **Estadísticas por episodio:** v3 las guarda en `meta/episodes/*.parquet` como `stats/{feature}/{min,max,mean,std,count,q01..q99}`, así que no hace falta recalcularlas para el chequeo 6.1.
  - Ojo: en v0.4.x los cuantiles globales se agregan como promedio ponderado de los cuantiles por episodio, que es una aproximación. En main pasaron a ser una envolvente conservadora. Importa si en algún momento normalizamos por `QUANTILES`, que es lo que usa π0.5.
- **Pipeline de Unitree** ([unitree_IL_lerobot](https://github.com/unitreerobotics/unitree_IL_lerobot)):
  1. Visualizar con `lerobot_dataset_viz.py`.
  2. Limpiar con el editor PyQt `data_editor`: "cut off excess segments… delete the bad episodes".
  3. Renumerar los episodios.
  4. Convertir.
  5. Evaluar offline con `eval_g1_dataset.py`.
  6. **Reproducir en el robot** con `replay_robot.py`.

  `xr_teleoperate` también tiene un `RerunLogger` para ver la grabación en vivo.
- **GR00T** ([real_world_deployment.md](https://github.com/NVIDIA/Isaac-GR00T/blob/main/getting_started/real_world_deployment.md)): es la lista más concreta, con filtros automáticos (latencia por cámara, saltos de estado), filtros manuales (desincronización, borrosidad, fracasos, caminos redundantes) y preprocesamiento (alinear timestamps, recortar extremos, dividir en subtareas).
- **DROID / Open X-Embodiment:**
  - DROID registra éxito o fracaso por episodio, con un aviso de la interfaz que lo pide. Publica unos 16.000 episodios fallidos aparte y no los usa para entrenar.
  - OXE estandariza al formato RLDS sin filtrar por calidad.
  - No encontré una herramienta pública de chequeo de calidad en ninguno de los dos más allá de su visualizador.

---

## 4. Estructura propuesta del notebook de validación

Orden lógico de lo básico a lo avanzado. Cada sección termina con una celda de **"veredicto"** que agrega columnas a una tabla `episode_flags` (una fila por episodio). La lógica vive en `toolkit/dataset/inspect.py`, siguiendo la convención del repo de que los notebooks sólo llaman y grafican.

0. **Configuración y carga**
   - Versión de LeRobot.
   - `LeRobotDataset(root=..., video_backend="pyav")`.
   - Tabla de trazabilidad `episode_index → episodio crudo → orden`.
   - Criterios de aprobación declarados *antes* de mirar los datos.
1. **Inventario y esquema** (1.1, 1.2): conteos, nombres, NaN, `stats.json` con cuantiles.
2. **Integridad temporal y de video** (1.3, 1.4, 5.4):
   - Timestamps: dejar constancia de que son sintéticos.
   - Video frente a parquet.
   - Cuadros congelados.
3. **Señales articulares** (2.1–2.5): saltos, ruido, saturación, dimensiones congeladas, distribuciones. Heatmaps de episodio × dimensión.
4. **Acción frente a estado** (3.1–3.3): lag por articulación, error de seguimiento (manos), chequeo de "acción = copia del estado", autocorrelación y chunk.
5. **Calidad por demostración** (4.1–4.6): duración, recorte de extremos, pausas, éxito (manual + indicadores), reintentos, consistencia y estrategias (trayectorias superpuestas, clusters).
6. **Calidad visual** (5.1–5.3, 5.5): desenfoque, exposición, visibilidad, sincronía cámara–estado. Grillas de cuadros.
7. **Deriva por orden de grabación** (6.1–6.3): gráficos de control, change-points (arreglo del pulgar), impacto en la normalización.
8. **Diversidad y cobertura** (7.1–7.3): condiciones iniciales, PCA del espacio de estados, casi duplicados.
9. **Split y fuga** (8.1–8.5):
   - Validar el split actual.
   - Comparar el split intercalado con uno de bloque temporal.
   - Distancia del held-out al train.
   - Normalización sólo con train.
10. **Curación avanzada (opcional):** DemInf sobre los episodios ya filtrados; comparar su ranking con las banderas heurísticas.
11. **Resumen y decisión:**
    - Tabla `episode_flags` con colores.
    - Lista de episodios a excluir o recortar con su motivo.
    - Comando `lerobot-edit-dataset` generado.
    - Exportar `split.json` versionado.
    - **Inspección visual obligatoria** de cada episodio marcado antes de excluirlo.

Gráficos estándar mínimos:
- histograma de duraciones;
- heatmap de episodio × articulación (std, máximo |Δ|, % de saturación);
- trayectorias superpuestas normalizadas en el tiempo;
- curva de lag estado–acción;
- gráfico de control por orden de grabación;
- PCA de condiciones iniciales y estados;
- grilla de primeros y últimos cuadros;
- CDF de distancia del held-out al train.

---

## 5. No verificado o con reservas

- Blog de Hugging Face "LeRobot Community Datasets" y paper de SmolVLA: los leí a través de un resumen automático y no los abrí completos. Lo que menciono de ahí (iluminación estable, ≥ 480×640, ~30 fps) puede estar parafraseado.
- La página del blog de π\*0.6 devolvió HTTP 429; usé el paper de arXiv.
- No pude confirmar:
  - los criterios de "low-quality" de GR00T N1 para datos reales;
  - si ALOHA Unleashed descartó los episodios fallidos;
  - la regla exacta de UMI para descartar demos con fallas de SLAM;
  - el método de normalización de Octo.
- Números marcados *(aprox.)* en la tabla de la sección 2: CUPID en Franka, Demo-SCORE en ALOHA real y con 10 rollouts, y QoQ.
- La sonda sobre nuestros datos fue una lectura rápida de los JSON crudos, sin repetición ni inspección visual:
  - El "desfase de las manos por contacto" es una hipótesis.
  - Que `episode_0008` sea "anterior al arreglo del pulgar" es consistente con los datos pero no está confirmado.
- No probé el Space del visualizador con un dataset local.
- Los umbrales sugeridos (8·MAD, 0,05 rad, 15 cuadros, CV de 0,2/0,4) son puntos de partida y hay que calibrarlos mirando los episodios.

---

## 6. Fuentes

**Papers**
- π0 — https://arxiv.org/abs/2410.24164
- π0.5 — https://arxiv.org/abs/2504.16054
- π\*0.6 / RECAP — https://arxiv.org/abs/2511.14759
- RT-1 — https://arxiv.org/abs/2212.06817
- ALOHA Unleashed — https://arxiv.org/abs/2410.13126
- ALOHA 2 — https://arxiv.org/abs/2405.02292
- Gemini Robotics — https://arxiv.org/abs/2503.20020
- TRI LBM, "A Careful Examination of Large Behavior Models" — https://arxiv.org/abs/2507.05331
- Diffusion Policy — https://arxiv.org/abs/2303.04137
- GR00T N1 — https://arxiv.org/abs/2503.14734
- DROID — https://arxiv.org/abs/2403.12945 (sitio: https://droid-dataset.github.io/)
- Open X-Embodiment — https://arxiv.org/abs/2310.08864
- Octo — https://arxiv.org/abs/2405.12213
- ACT / ALOHA — https://arxiv.org/abs/2304.13705
- Mobile ALOHA — https://arxiv.org/abs/2401.02117
- UMI — https://arxiv.org/abs/2402.10329
- DemInf — https://arxiv.org/abs/2502.08623
- CUPID — https://arxiv.org/abs/2506.19121
- DataMIL — https://arxiv.org/abs/2505.09603
- Demo-SCORE — https://arxiv.org/abs/2503.03707
- SCIZOR — https://arxiv.org/abs/2505.22626
- QoQ — https://arxiv.org/abs/2603.09056
- Data Quality in Imitation Learning — https://arxiv.org/abs/2306.02437
- robomimic — https://arxiv.org/abs/2108.03298 (estudio: https://robomimic.github.io/study/)
- Re-Mix — https://arxiv.org/abs/2408.14037
- Data Scaling Laws in Imitation Learning — https://arxiv.org/abs/2410.18647
- Shortcut Learning in Generalist Robot Policies — https://arxiv.org/abs/2508.06426
- AWE — https://arxiv.org/abs/2307.14326
- S2I (selección por segmentos) — https://arxiv.org/abs/2409.19917
- Compatibilidad entre demostradores (Gandhi et al.) — https://arxiv.org/abs/2210.08073
- SmolVLA — https://arxiv.org/abs/2506.01844
- DINOv2 (deduplicación) — https://arxiv.org/abs/2304.07193
- Pech-Pacheco et al. 2000 (foco por Laplaciano) — https://dblp.org/rec/conf/icpr/Pech-PachecoCCF00.html
- SPARC (suavidad) — https://link.springer.com/article/10.1186/s12984-015-0090-9

**Documentación y repositorios**
- LeRobot dataset v3 — https://huggingface.co/docs/lerobot/lerobot-dataset-v3
- LeRobot, grabación y consejos — https://huggingface.co/docs/lerobot/il_robots
- LeRobot, herramientas de dataset — https://huggingface.co/docs/lerobot/using_dataset_tools
- Repo de LeRobot — https://github.com/huggingface/lerobot (archivos: `datasets/compute_stats.py`, `datasets/utils.py`, `datasets/video_utils.py`, `scripts/lerobot_edit_dataset.py`, `scripts/lerobot_dataset_viz.py`; v0.3.3 `check_timestamps_sync`)
- LeRobot Dataset Visualizer — https://huggingface.co/spaces/lerobot/visualize_dataset (código: https://github.com/huggingface/lerobot-dataset-visualizer)
- Blog de Hugging Face sobre datasets — https://huggingface.co/blog/lerobot-datasets
- Rerun + LeRobot — https://rerun.io/docs/howto/logging-and-ingestion/lerobot
- Despliegue real de Isaac-GR00T — https://github.com/NVIDIA/Isaac-GR00T/blob/main/getting_started/real_world_deployment.md (más `data_preparation.md`, `data_config.md` y `finetune_new_embodiment.md` en la misma carpeta)
- Unitree IL LeRobot — https://github.com/unitreerobotics/unitree_IL_lerobot
- Unitree xr_teleoperate — https://github.com/unitreerobotics/xr_teleoperate
- ACT (código) — https://github.com/tonyzhaozh/act
- Diffusion Policy (código) — https://github.com/real-stanford/diffusion_policy
