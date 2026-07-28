# Arquitectura de control para la demo completa del G1

Estado: decisión de diseño, 28-jul-2026.

Este documento fija qué sistema estamos construyendo y qué condiciones deben
mantenerse verdaderas durante el desarrollo. La simulación es el banco de
pruebas del futuro G1 EDU físico; una solución que dependa de que todos los
procesos vivan en la misma computadora o de que la red nunca falle no sirve.

## Resultado final

La misión completa es:

1. Una persona da una orden hablada.
2. El agente entiende la intención y arma un plan con capacidades conocidas.
3. El robot navega hasta una pose desde la cual pueda ver el reloj.
4. La percepción lee la hora desde la imagen.
5. El robot navega hasta una pose de aproximación a la mesa.
6. Una alineación fina coloca cuerpo, manos y objeto dentro del margen de
   captura del controlador de agarre.
7. El controlador de agarre toma el objeto y confirma que sigue sujeto.
8. El robot transporta el objeto mientras camina.
9. Busca activamente a la persona correcta, mantiene una distancia segura y
   se orienta hacia ella.
10. Entrega el objeto y confirma el resultado.

La misión no termina cuando el agente publica el último paso. Termina cuando
cada capacidad reportó un resultado medido y el objeto fue entregado sin
caídas, colisiones ni pérdida silenciosa de control.

## Computadoras y enlaces que deben sobrevivir al cambio a hardware

| Lugar | Responsabilidad | Qué pasa si se corta su enlace |
|---|---|---|
| Computadora de control | equilibrio, locomoción, límites y parada segura | mantiene o detiene el cuerpo sin depender de la red |
| Jetson a bordo | autoridad de movilidad, navegación, percepción y control de brazos | conserva una conducta segura local |
| Servidor externo | planificación semántica y modelos grandes | la misión se pausa; el robot no pierde equilibrio ni sigue caminando |

El servidor nunca debe ser parte de un lazo que necesite respuesta rápida. La
latencia y los cortes simulados son una prueba funcional, no una decoración.

## Tres capas que no se deben mezclar

### 1. Modo físico del robot

Decide qué controlador sostiene el cuerpo:

- `PASSIVE`: motores amortiguados; no promete sostenerse de pie.
- `STAND`: postura y equilibrio activos.
- `VELOCITY`: locomoción activa siguiendo una velocidad deseada.
- `FAULT`: salida segura después de una falla.
- `FROZEN`: sólo simulación; reescribe el estado para iterar sin recargar.

Unitree usa la misma separación conceptual en su despliegue:
`Passive -> FixStand -> Velocity`. El G1 real podrá reemplazar nuestros
controladores por los propietarios sin cambiar las capas superiores.

### 2. Autoridad sobre la movilidad

Decide quién puede pedir velocidades. Es un recurso exclusivo: puede tener un
solo dueño a la vez.

Fuentes previstas:

- `STAND`: mantener una pose cuando corresponde.
- `NAVIGATION`: seguir un objetivo del mapa.
- `MANUAL`: operador humano.
- `TEST`: banco de pruebas.

Cada fuente publica en su propio topic. Un único árbitro selecciona la fuente
autorizada y es el único que alimenta la cadena final de seguridad.

### 3. Ejecución de la misión

Decide qué capacidad ejecutar y en qué orden. Navegar y manipular son acciones
largas: deben aceptar cancelación, publicar progreso y devolver un resultado.
La misión puede usar movilidad y brazos al mismo tiempo; por eso
`MANIPULATE` no debe ser un modo global que sustituya a `STAND`.

## Camino único del comando de movilidad

```text
/g1/cmd_vel/stand       \
/g1/cmd_vel/navigation   \
/g1/cmd_vel/manual        -> mobility_authority
/g1/cmd_vel/test         /          |
                                   v
                           velocity_smoother
                                   |
                                   v
                           collision_monitor
                                   |
                                   v
                               /cmd_vel
                                   |
                                   v
                         robot / Unitree API
```

Sólo el último filtro publica en `/cmd_vel`. Mientras todavía no exista el
filtro de colisiones, `mobility_authority` ocupa temporalmente ese último lugar.

El árbitro aplica una concesión con vencimiento:

- una fuente solicita control y espera confirmación;
- los comandos frescos renuevan la concesión;
- al vencer, la salida pasa a cero y el dueño se libera;
- toda transición publica dueño anterior, dueño nuevo y motivo;
- un comando de una fuente no autorizada se descarta y se cuenta;
- seguridad puede cortar cualquier dueño, pero no cuenta como otra fuente de
  movimiento.

Este patrón coincide con:

- `ros2_control`, que impide activar controladores con interfaces de comando
  ya reclamadas;
- las concesiones de Spot, exclusivas por recurso y renovadas periódicamente;
- `twist_mux`, que separa entradas de velocidad y selecciona una por prioridad
  y antigüedad;
- la cadena de Nav2, donde el monitor de colisiones es el último publicador de
  velocidad.

No hay una interfaz ROS 2 estándar para una concesión de movilidad de alto
nivel. En la primera integración se puede transportar la solicitud con
mensajes estándar; antes del hardware se definirá una interfaz tipada propia.
El comportamiento y las pruebas no dependen de ese formato.

## Qué significa quedarse quieto

Hay dos lazos distintos:

1. El equilibrio mantiene el cuerpo de pie respecto de sus apoyos.
2. El mantenimiento de pose corrige la posición y orientación respecto de
   odometría.

El primero debe estar activo siempre que el robot esté de pie. El segundo sólo
se activa cuando la tarea lo necesita y la estimación de pose es válida.
Navegación no mantiene posición después de terminar: entrega el recurso.

Durante un agarre no se corrige una deriva caminando en silencio. Si la base
sale del sobre permitido, se pausa el brazo, se asegura el objeto si ya está
sujeto, se reposiciona explícitamente y se reintenta.

## Márgenes iniciales y cómo validarlos

| Contexto | Error de posición | Error de orientación | Acción |
|---|---:|---:|---|
| Tránsito en sala | 20–25 cm | 10° | aceptar llegada gruesa |
| Espera libre | 10 cm | 5° | corregir con velocidad limitada |
| Aproximación a mesa | 8–10 cm | 5° | terminar navegación |
| Alineación para agarrar | 3–5 cm | 2–3° | habilitar el agarre |
| Durante el agarre | máximo 5 cm | máximo 5° | abortar y reposicionar |
| Cerca de una persona | distancia 0,8–1,0 m | 10° | no perseguir un ancla del mapa |

Son requisitos iniciales, no números sagrados. Cada uno se reemplaza por un
valor medido cuando conozcamos el margen real de percepción y agarre.

Para considerar estable una espera:

- medir al menos 30 s de tiempo simulado después del transitorio inicial;
- informar error medio, percentil 95 y máximo, no sólo posición final;
- verificar que no haya un ciclo de ida y vuelta creciente;
- registrar velocidad pedida y real;
- repetir al menos tres veces si el experimento incluye contactos.

## Invariantes de seguridad y observabilidad

1. Hay como máximo un publicador en el último topic de velocidad.
2. Todo comando tiene fuente identificable y edad conocida.
3. Perder Jetson, servidor o dueño de movilidad produce una salida segura.
4. Un salto de localización invalida el objetivo; nunca provoca caminar hacia
   un ancla vieja.
5. Una acción terminada no sigue publicando.
6. Una acción cancelada confirma la cancelación antes de ceder el recurso.
7. El tablero muestra modo físico, dueño de movilidad, última transición,
   comando seleccionado y comando descartado.
8. Las pruebas usan la misma interfaz de autoridad que la misión; no publican
   directamente al robot.

## Orden de integración

1. Autoridad única sobre movilidad, sin cambiar física.
2. Medición limpia de la deriva sin navegación ni mantenimiento de pose.
3. Separación de `STAND` y `VELOCITY` en el supervisor de locomoción.
4. Navegación como acción cancelable.
5. Alineación fina y contrato de manipulación.
6. Búsqueda activa de persona y protección de distancia.
7. Sustitución de reloj fijo, detector de color y agarre simulado por los
   componentes finales.

## Referencias primarias

- ROS 2, acciones para tareas largas:
  https://docs.ros.org/en/rolling/Concepts/Basic/About-Actions.html
- ROS 2 Control, reclamo exclusivo y encadenamiento de controladores:
  https://control.ros.org/jazzy/doc/ros2_control/controller_manager/doc/controller_chaining.html
- Nav2, cadena final de velocidades y monitor de colisiones:
  https://docs.nav2.org/tutorials/docs/using_collision_monitor.html
- Nav2, navegación expuesta como acción:
  https://docs.nav2.org/configuration/packages/configuring-bt-navigator.html
- Boston Dynamics, concesiones de control:
  https://dev.bostondynamics.com/docs/concepts/lease_service.html
- Unitree RL Lab, despliegue y validación sim-to-sim:
  https://github.com/unitreerobotics/unitree_rl_lab
- Unitree SDK2, separación de control de alto y bajo nivel:
  https://github.com/unitreerobotics/unitree_sdk2_python
- Selector de velocidades `twist_mux`:
  https://github.com/ros-teleop/twist_mux
