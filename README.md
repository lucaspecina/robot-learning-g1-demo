# robot-learning-g1-demo

Laboratorio para aprender robot learning con un Unitree G1 EDU (humanoide), en
simulación y en el robot real. Tiene dos partes que se conectan en un solo
punto:

- **Aprender a producir políticas** — `learning/`, `toolkit/`, `robot/`. Es el
  foco actual.
- **La demo de punta a punta** — `g1/`. Una persona le habla al robot; el robot
  guarda dónde empezó, lee un reloj, elige la mesa que corresponde a la hora,
  navega hasta ella, toma el objeto y vuelve. Está viva pero en pausa.

El punto de conexión es la habilidad `grasp_object` de la demo
([`g1/skill_catalog.py`](g1/skill_catalog.py)). Hoy es un hueco marcado
`placeholder`; cuando una política entrenada acá pase su evaluación, la demo
la usa para agarrar.

## Cómo se aprende acá

El orden lo marca el flujo de trabajo, no el temario:

1. **Primero el flujo completo con ACT**: grabar, convertir al formato de
   LeRobot, entrenar, validar offline, probar en simulación y hacer un
   simulacro en el robot. ACT es la primera política, ya entrenada sobre
   episodios reales de teleoperación.
2. **Después, otros modelos como experimentos**: Diffusion Policy, VLAs. Mismos
   datos, misma validación, una fila más en la tabla de comparación.
3. **La profundidad, a demanda**: hiperparámetros, papers y derivaciones
   cuando un resultado lo pide.

El curso de referencia es Robot Learning de ETH Zürich; la clase de cada
modelo se ve justo antes de usarlo.

## Estructura

```
learning/   un experimento por modelo: act/, diffusion_policy/, vla/
toolkit/    herramientas reusables: inspeccionar datasets, evaluar, comparar
robot/      recetas del G1 real: teleoperación, conversión, parches
g1/         la demo de punta a punta (navegación, percepción, agente, skills)
systems/    la Jetson y el servidor simulados, con red degradable
scripts/    instalar el entorno de aprendizaje; encender y apagar la VM
docs/       contexto: simuladores, ecosistema, arquitectura, glosario
notas/      fuera de Git: lo interno que no va en un repo público
```

## Empezar

```bash
bash scripts/setup_learning.sh     # entorno de Python con LeRobot (Mac, VM o workstation)
source .venv/bin/activate
pytest
```

Cómo seguir: [`learning/act/`](learning/act/) y [`toolkit/`](toolkit/).

## La demo en la VM de simulación

La demo corre en Isaac Sim, en una VM de Azure con GPU. Los nombres de los
recursos de Azure no se versionan: copiar
[`scripts/lab.env.example`](scripts/lab.env.example) a `scripts/lab.env` y
completarlo.

```bash
bash scripts/lab_up.sh      # GitHub -> VM -> simulador -> túnel (la terminal queda de túnel)
bash scripts/lab_down.sh    # cierra todo y desasigna la VM (deja de facturar)
bash scripts/lab_sync.sh    # sólo sincroniza una VM ya encendida
```

`lab_sync.sh` exige que el commit local ya esté en GitHub y que el clon de la
VM no tenga cambios; lo actualiza sólo con un avance directo de Git. Cómo se
arma la VM desde cero: [`scripts/vm/README.md`](scripts/vm/README.md).

Estado de la demo: [`g1/README.md`](g1/README.md). Integración con el robot
físico: [`g1/REAL_ROBOT_INTEGRATION.md`](g1/REAL_ROBOT_INTEGRATION.md).

## Contexto

- [`docs/06_estado_y_estrategias.md`](docs/06_estado_y_estrategias.md) — mapa
  de situación y estrategias de manipulación.
- [`docs/05_workflow_de_entrenamiento.md`](docs/05_workflow_de_entrenamiento.md)
  — el flujo de entrenamiento del ecosistema NVIDIA.
- [`docs/00_mapa_simuladores.md`](docs/00_mapa_simuladores.md),
  [`docs/01_unitree_ecosistema_datos.md`](docs/01_unitree_ecosistema_datos.md),
  [`docs/02_arquitectura_inteligencia.md`](docs/02_arquitectura_inteligencia.md),
  [`docs/03_arquitectura_inteligencia_humanoide.md`](docs/03_arquitectura_inteligencia_humanoide.md),
  [`docs/04_generacion_de_datos.md`](docs/04_generacion_de_datos.md),
  [`docs/07_receta_teleoperacion.md`](docs/07_receta_teleoperacion.md),
  [`docs/GLOSARIO.md`](docs/GLOSARIO.md).

Reglas de trabajo y convenciones: [`AGENTS.md`](AGENTS.md).

La fase inicial con el Go2 en MuJoCo se retiró el 2-oct-2026; sigue en el
historial de Git.
