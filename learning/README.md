# learning/

Un experimento por modelo. Todos usan los mismos datos y se evalúan con las
mismas herramientas de [`toolkit/`](../toolkit/), para que la comparación sea
justa.

| Carpeta | Modelo | Estado |
|---|---|---|
| [`act/`](act/) | ACT (Action Chunking with Transformers) | primera política, entrenada fuera del repo; falta reproducirla acá |
| `diffusion_policy/` | Diffusion Policy | después de cerrar el flujo con ACT |
| `vla/` | un VLA chico del ecosistema LeRobot; GR00T si hay GPU alquilada | después |

Cada experimento declara **antes** de entrenar: qué pregunta responde, qué
métrica debería mejorar y cuál es el criterio de aprobación (ver
[`AGENTS.md`](../AGENTS.md)).

RL y world models quedan para más adelante: tienen otra cadena (simulación y
recompensa en vez de datos grabados) y hoy ninguna tarea del G1 los pide.

## Ideas propias

Ideas de arquitectura para mejorar las políticas, antes de convertirse en
experimentos: [`ideas/`](ideas/).

- [`ideas/memoria_e_historia.md`](ideas/memoria_e_historia.md): seis formas de
  darle memoria a ACT sin caer en la confusión causal.
