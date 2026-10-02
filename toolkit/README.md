# toolkit/

Herramientas reusables con cualquier política y en cualquier máquina (Mac, VM,
Jetson). El código vive en módulos; los notebooks sólo los llaman y grafican.

Se arma sacando lo genérico del notebook de evaluación de ACT, que ya funciona.

| Herramienta | Pregunta que responde | Estado |
|---|---|---|
| `dataset/inspect` | ¿Los episodios están sanos? Duración, saltos, cámaras, estadísticas | por traer del notebook de ACT |
| `dataset/split` | ¿Qué episodios quedan fuera del entrenamiento para evaluar? | por hacer |
| `eval/offline` | ¿Predice bien sobre episodios que no vio? Error por articulación | por traer del notebook de ACT |
| `eval/rollout_sim` | ¿Cumple la tarea cuando sus propios errores se acumulan? | por hacer |
| `eval/dry_run` | ¿Qué haría en el robot sin moverlo? | por traer de `eval_g1.py` |
| `checks` | ¿Carga con su pre y postprocesador? ¿La latencia entra en el ciclo? | por hacer |
| `compare` | ¿Esta política es mejor que la anterior, con las mismas métricas? | por hacer |

La evaluación offline usa *teacher forcing*: en cada paso la política recibe la
observación verdadera, no la que resultó de sus propias acciones. Mide si
aprendió, pero no si la tarea sale; para eso está el rollout en lazo cerrado.
