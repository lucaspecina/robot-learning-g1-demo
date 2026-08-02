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
ROOM_WALL_THICKNESS = 0.12
# Los límites describen el centro de cada pared porque Isaac coloca allí los
# cubos. Distancias y seguridad deben usar la cara que mira hacia la sala.
ROOM_INTERIOR_BOUNDS = {
    "xmin": WORLD_BOUNDS["xmin"] + ROOM_WALL_THICKNESS / 2.0,
    "xmax": WORLD_BOUNDS["xmax"] - ROOM_WALL_THICKNESS / 2.0,
    "ymin": WORLD_BOUNDS["ymin"] + ROOM_WALL_THICKNESS / 2.0,
    "ymax": WORLD_BOUNDS["ymax"] - ROOM_WALL_THICKNESS / 2.0,
}

SCENE_POSITIONS = {
    "clock": (0.0, 2.5),
    "red_table": (4.0, 2.6),
    "blue_table": (4.0, -2.6),
    "navigation_crate": (1.5, 1.0),
}

# Una meta de navegación es el lugar seguro desde el cual usar un objeto, no
# el centro del objeto. Ir al centro de la mesa haría caminar al robot contra
# el mueble.
NAVIGATION_TARGETS = {
    "reloj": (0.8, 1.8, 2.4228),
}

TABLE_SIZE = (0.8, 1.2)

# Este cajón corta exactamente la recta entre el origen y la meta de prueba,
# pero deja libres el corredor y las aproximaciones de la misión. Es geometría
# física y visible al LiDAR; Nav2 no recibe su posición por este módulo.
NAVIGATION_TEST_OBSTACLE = {
    "id": "navigation_crate",
    "label": "obstáculo de navegación",
    "x": SCENE_POSITIONS["navigation_crate"][0],
    "y": SCENE_POSITIONS["navigation_crate"][1],
    "size_x": 0.6,
    "size_y": 0.6,
    # La altura normal conserva la referencia del ensayo 2D. Las pruebas pueden
    # bajarla a 45 cm para exigir que la cámara de profundidad cubra el hueco
    # que el plano horizontal no ve.
    "height": 1.8,
    "rgb": (0.72, 0.46, 0.12),
}
NAVIGATION_TEST_GOAL = (3.0, 2.0, 0.0)

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
        {
            "id": NAVIGATION_TEST_OBSTACLE["id"],
            "label": NAVIGATION_TEST_OBSTACLE["label"],
            "shape": "rectangle",
            "x": NAVIGATION_TEST_OBSTACLE["x"],
            "y": NAVIGATION_TEST_OBSTACLE["y"],
            "size_x": NAVIGATION_TEST_OBSTACLE["size_x"],
            "size_y": NAVIGATION_TEST_OBSTACLE["size_y"],
            "color": "#b8751f",
        },
    ],
}
