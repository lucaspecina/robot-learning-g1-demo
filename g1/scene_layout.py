#!/usr/bin/env python3
"""Descripción compartida de la habitación de la demo.

Isaac, el agente y el tablero consumen estos mismos datos. Mantener una sola
fuente evita que el mapa muestre un mundo distinto del que realmente contiene
el simulador.
"""

WORLD_BOUNDS = {
    "xmin": -1.5,
    "xmax": 5.5,
    "ymin": -3.5,
    "ymax": 4.0,
}

SCENE_POSITIONS = {
    "clock": (0.0, 2.5),
    "red_table": (4.0, 2.6),
    "blue_table": (4.0, -2.6),
}

# Una meta de navegación es el lugar seguro desde el cual usar un objeto, no
# el centro del objeto. Ir al centro de la mesa haría caminar al robot contra
# el mueble.
NAVIGATION_TARGETS = {
    "reloj": (0.8, 1.8, 2.4228),
}

TABLE_SIZE = (0.8, 1.2)

COLORED_TABLES = (
    {
        "id": "red_table",
        "label": "mesa A roja",
        "rgb": (0.75, 0.12, 0.12),
    },
    {
        "id": "blue_table",
        "label": "mesa B azul",
        "rgb": (0.12, 0.20, 0.75),
    },
)

# Esta forma simple y serializable se incrusta en la página web. Las medidas
# son las mismas que usa Isaac, por lo que el dibujo no necesita constantes
# duplicadas ni aproximaciones ocultas.
DASHBOARD_SCENE = {
    "world": WORLD_BOUNDS,
    "landmarks": [
        {
            "id": "clock",
            "label": "reloj",
            "shape": "circle",
            "x": SCENE_POSITIONS["clock"][0],
            "y": SCENE_POSITIONS["clock"][1],
            "radius": 0.12,
            "color": "#d8d8c8",
        },
        *[
            {
                "id": table["id"],
                "label": table["label"],
                "shape": "rectangle",
                "x": SCENE_POSITIONS[table["id"]][0],
                "y": SCENE_POSITIONS[table["id"]][1],
                "size_x": TABLE_SIZE[0],
                "size_y": TABLE_SIZE[1],
                "color": (
                    "#9f2929"
                    if table["id"] == "red_table"
                    else "#3354bf"
                ),
            }
            for table in COLORED_TABLES
        ],
    ],
}
