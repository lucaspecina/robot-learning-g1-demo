#!/usr/bin/env python3
"""Catálogo cerrado de capacidades que puede usar el planificador."""
from copy import deepcopy


INITIAL_WORLD_FACTS = [
    "robot_pose_known",
    "clock_location_known",
]


SKILL_CATALOG = [
    {
        "name": "remember_home",
        "description": (
            "Guarda la posición y orientación actuales para poder regresar "
            "al mismo lugar al final de la misión."
        ),
        "availability": "ready",
        "variants": [
            {
                "argument": None,
                "argument_description": "No recibe argumento.",
                "preconditions": ["robot_pose_known"],
                "effects": ["home_saved"],
            }
        ],
    },
    {
        "name": "navigate_to",
        "description": (
            "Lleva la base del robot hasta un destino ya localizado y espera "
            "la confirmación de llegada. No encuentra objetos ni planifica "
            "alrededor de obstáculos todavía."
        ),
        "availability": "ready",
        "variants": [
            {
                "argument": "clock",
                "argument_description": (
                    "Usa la pose de observación conocida del reloj."
                ),
                "preconditions": ["clock_location_known"],
                "effects": ["at_clock"],
            },
            {
                "argument": "home",
                "argument_description": (
                    "Regresa a la pose guardada por remember_home."
                ),
                "preconditions": ["home_saved"],
                "effects": ["at_home"],
            },
        ],
    },
    {
        "name": "look_at",
        "description": (
            "Confirma con la cámara que el objetivo indicado está visible. "
            "Hoy no mueve activamente la cabeza ni busca fuera del cuadro."
        ),
        "availability": "ready",
        "variants": [
            {
                "argument": "clock",
                "argument_description": "Confirma visualmente el reloj.",
                "preconditions": ["at_clock"],
                "effects": ["clock_confirmed"],
            }
        ],
    },
    {
        "name": "read_clock",
        "description": (
            "Envía el recorte reciente del reloj al modelo visual remoto, "
            "valida la respuesta y guarda una hora estructurada."
        ),
        "availability": "ready",
        "variants": [
            {
                "argument": None,
                "argument_description": "No recibe argumento.",
                "preconditions": ["clock_confirmed"],
                "effects": ["clock_reading_known"],
            }
        ],
    },
    {
        "name": "choose_table",
        "description": (
            "Aplica la regla exacta de la demo: antes de las 12:00 elige la "
            "mesa A roja; a las 12:00 o después elige la mesa B azul. La "
            "comparación la hace código determinista, no el LLM."
        ),
        "availability": "ready",
        "variants": [
            {
                "argument": None,
                "argument_description": "No recibe argumento.",
                "preconditions": ["clock_reading_known"],
                "effects": ["selected_table_known"],
            }
        ],
    },
    {
        "name": "search_table",
        "description": (
            "Pide detectar la mesa elegida, verifica su color y usa "
            "profundidad para ubicarla. Sólo analiza la vista actual; sirve "
            "cuando la mesa ya debería estar delante del robot."
        ),
        "availability": "ready",
        "variants": [
            {
                "argument": "$selected_table",
                "argument_description": (
                    "Referencia a la mesa roja o azul elegida por choose_table."
                ),
                "preconditions": ["selected_table_known"],
                "effects": ["table_location_known"],
            }
        ],
    },
    {
        "name": "scan_for_table",
        "description": (
            "Busca activamente la mesa elegida alrededor del robot. Analiza "
            "vistas superpuestas y gira el cuerpo mediante una Action "
            "cancelable. El detector local o una señal amplia del color "
            "filtran candidatos baratos; ninguno declara éxito. Sólo "
            "Grounding DINO confirma la mesa y profundidad calcula su "
            "posición. Queda en STAND entre movimientos y se detiene mirando "
            "la mesa cuando la encuentra."
        ),
        "availability": "ready",
        "variants": [
            {
                "argument": "$selected_table",
                "argument_description": (
                    "Referencia a la mesa roja o azul elegida por choose_table."
                ),
                "preconditions": ["selected_table_known"],
                "effects": ["table_location_known"],
            }
        ],
    },
    {
        "name": "approach_table",
        "description": (
            "Calcula una pose de preaproximación sobre la línea de visión, "
            "navega hasta ella, queda mirando la mesa y vuelve a medirla. "
            "No afirma que la base ya esté alineada para agarrar."
        ),
        "availability": "ready",
        "variants": [
            {
                "argument": "$selected_table",
                "argument_description": (
                    "La mesa localizada por search_table o scan_for_table."
                ),
                "preconditions": ["table_location_known"],
                "effects": ["at_table_staging"],
            }
        ],
    },
    {
        "name": "set_arm_pose",
        "description": (
            "Mueve los brazos a una postura conocida y espera confirmación "
            "por medición de articulaciones."
        ),
        "availability": "ready",
        "variants": [
            {
                "argument": "ready",
                "argument_description": (
                    "Libera la cámara y prepara los brazos para agarrar."
                ),
                "preconditions": ["at_table_staging"],
                "effects": ["arms_ready"],
            },
            {
                "argument": "transport",
                "argument_description": (
                    "Coloca los brazos en la postura prevista para transportar."
                ),
                "preconditions": ["object_grasped"],
                "effects": ["transport_pose_set"],
            },
        ],
    },
    {
        "name": "align_with_table",
        "description": (
            "Ajusta finamente la base usando nuevas mediciones de la mesa y "
            "del objeto hasta quedar dentro de la tolerancia de agarre. El "
            "contrato existe, pero el control visual todavía no está "
            "implementado."
        ),
        "availability": "placeholder",
        "variants": [
            {
                "argument": "$selected_table",
                "argument_description": (
                    "Mesa elegida y confirmada desde la preaproximación."
                ),
                "preconditions": ["at_table_staging", "arms_ready"],
                "effects": ["at_selected_table"],
            }
        ],
    },
    {
        "name": "grasp_object",
        "description": (
            "Agarra el objeto de la mesa. El contrato existe para planificar "
            "la misión completa, pero la policy de agarre todavía no está "
            "implementada."
        ),
        "availability": "placeholder",
        "variants": [
            {
                "argument": None,
                "argument_description": "No recibe argumento.",
                "preconditions": ["at_selected_table", "arms_ready"],
                "effects": ["object_grasped"],
            }
        ],
    },
]


def skill_catalog_for_model() -> list[dict]:
    """Entrega una copia porque la respuesta remota nunca debe mutar el catálogo."""
    return deepcopy(SKILL_CATALOG)
