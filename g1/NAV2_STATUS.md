# Estado de la navegación Nav2

Fecha de corte: 2-ago-2026.

## Qué quedó integrado

La misión conserva `/g1/navigate_to_pose` y `/g1/spin`. `nav2_adapter.py`
adquiere la autoridad exclusiva, reenvía la acción al Nav2 oficial y devuelve
el mando a `STAND` en éxito, cancelación o falla. Nunca publica velocidad.

Nav2 usa:

- NavFn para calcular la ruta sobre el mapa global;
- DWB para seguirla y evitar obstáculos sobre el mapa local;
- Velocity Smoother para limitar cambios bruscos;
- Behavior Server para giro, retroceso y espera;
- BT Navigator para coordinar ruta, seguimiento y recuperaciones;
- dos costmaps de 5 cm y Collision Monitor como barrera final.

La adaptación propia es el límite medido del G1: 0,30 m/s y 0,50 rad/s como
máximos, 0,10 m/s como velocidad útil mínima y huella cuadrada de 60 cm.

## Experimentos que decidieron el controlador

| Cambio único | Resultado |
|---|---|
| MPPI, 2.000 muestras | dos CPU saturadas; 1 m no llegó en 180 s de pared |
| MPPI, 1.000 muestras | siguió saturado; orden de 0,01–0,02 m/s, menor que la deriva |
| crítico oficial de velocidad muerta a 0,10 m/s | orden de ~0,05 m/s; no llegó |
| MPPI usando la orden anterior | ~0,06 m/s; no llegó |
| DWB con mínimo duro de 0,10 m/s | llegó a 0,50 m y volvió a `STAND` |
| espera interna 20 ms | dos éxitos y un aborto al aceptar una ruta bajo carga |
| espera interna 500 ms | repetición aprobada; no cambia movimiento ni seguridad |

La elección de DWB no afirma que sea universalmente mejor que MPPI. Es el
controlador oficial que cumple el presupuesto medido de esta Jetson simulada y
permite representar la velocidad mínima que esta policy realmente ejecuta.

Después de integrarlo al lanzador normal pasaron:

- quietud: 60 s, máximo 4 cm y final 3 cm;
- caminata directa: 2,16 m, desvío 4,6°, freno 0 cm, sin caída;
- sensor y mapas: vuelta de 359,4°, mapa global utilizable y obstáculo más
  cercano a 1,08 m con 35 cm libres alrededor del cuerpo.

## Lo que todavía no está aprobado

- rodear un obstáculo físico que corte la línea directa;
- repetición e inspección visual de ese rodeo;
- localización móvil transferible: Isaac todavía entrega odometría perfecta y
  la T4 no compensa bien el movimiento del barrido RTX;
- huella dinámica cuando brazos y carga sobresalen del torso.

## Referencias oficiales

- [Nav2: selección de algoritmos](https://docs.nav2.org/setup_guides/algorithm/select_algorithm.html)
- [Nav2: DWB](https://docs.nav2.org/configuration/packages/configuring-dwb-controller.html)
- [Nav2: mapa local y controlador](https://docs.nav2.org/configuration/packages/configuring-controller-server.html)
- [Nav2: BT Navigator y sus plazos](https://docs.nav2.org/configuration/packages/configuring-bt-navigator.html)
- [Nav2: huella completa](https://docs.nav2.org/configuration/packages/trajectory_critics/obstacle_footprint.html)
