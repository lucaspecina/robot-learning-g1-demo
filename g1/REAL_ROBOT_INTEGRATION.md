# Integración inicial con el G1 físico

Estado: **decisión de arquitectura y plan de validación; todavía no implementado**.
Fecha: 18-ago-2026.

Este documento define cómo trasladar la demo al G1 EDU sin sustituir de entrada
los controladores que mantienen el equilibrio. También registra qué partes del
repositorio ya son transferibles, cuáles son sólo de simulación y qué falta
medir antes de permitir movimiento autónomo.

## Decisión principal: quién controla las piernas

En la primera integración física, la locomoción queda a cargo del servicio de
alto nivel que ya ejecuta Unitree dentro del robot. Nuestro sistema entrega una
velocidad deseada —avance, desplazamiento lateral y giro— mediante
`LocoClient.Move(vx, vy, vyaw)`. El controlador propietario de Unitree resuelve
equilibrio, contacto de los pies, marcha y comandos de motores.

No llamamos a esto **política oficial de Unitree**: la implementación interna es
propietaria y no conocemos si es una política aprendida, un controlador clásico
o una combinación. Lo verificable y público es la interfaz de alto nivel
`LocoClient` y los ejemplos oficiales que usan `Move`, `StopMove`, `Damp` y
`ZeroTorque`.

La política WBC-AGILE que hoy mueve la simulación no se desplegará inicialmente
en las piernas del robot real. Sigue siendo la referencia de locomoción en
Isaac y un banco comparativo. Ejecutarla en hardware implicaría liberar o
reemplazar el controlador de locomoción del fabricante y validar desde cero el
modelo corporal, orden de articulaciones, signos, ganancias, frecuencia,
latencia, estimación de estado, contactos, límites y respuesta ante fallas. No
existe todavía una necesidad medida que justifique ese riesgo.

En esta etapa no se publicará `lowcmd` ni se llamará a
`MotionSwitcher.ReleaseMode`. Esas operaciones pertenecen a una futura línea
de control de bajo nivel, separada del primer despliegue.

## Estado real del repositorio hoy

La arquitectura superior está bastante avanzada, pero el último tramo hacia el
hardware todavía no existe. Que el tablero y la misión funcionen en Isaac no
significa que puedan conectarse directamente al G1 físico.

| Parte existente | Estado frente al G1 físico |
|---|---|
| Ejecutor de misión, Actions y fallas explícitas | transferible; falta validarlo con tiempos y fallas reales |
| `mobility_authority` | patrón reutilizable; falta integrar la autoridad real de app/control/autonomía |
| Nav2 y Collision Monitor | configuración inicial útil; mapa, odometría, LiDAR y huella deben medirse otra vez |
| Percepción y evidencia fechada | lógica reutilizable; entrada de cámara, profundidad y rendimiento son de simulación |
| Tablero | estructura reutilizable; faltan adaptadores reales y un camino de video eficiente |
| WBC-AGILE y `g1_robot.py` | locomoción y cuerpo de Isaac; no comandan el robot físico |
| Adaptador `/cmd_vel` → `LocoClient` | no implementado |
| Adaptadores de cámara, batería, articulaciones, IMU, LiDAR y odometría | no implementados o no validados para esta unidad |
| Parada ante pérdida de la computadora externa | no probada en hardware |

Los scripts históricos `00_read_lowstate.py`, `01_stand.py`,
`02_record_lowstate.py` y `03_walk_policy.py` usan mensajes y supuestos del Go2.
Su opción llamada `--real` **no los convierte en herramientas válidas para el
G1**. No deben ejecutarse contra este robot hasta reemplazarlos por interfaces
G1 explícitas y pasar la escalera de pruebas de este documento.

## Arquitectura objetivo inicial

```text
SERVIDOR EXTERNO            RUNTIME DE LA DEMO             COMPUTADORAS DEL G1
modelos grandes       ROS 2/Nav2/percepción/tablero       servicios del fabricante
      |                            |                              |
      +------ pedidos lentos -----+                              |
                                   \                             |
                                    mobility_authority            |
                                             |                    |
                                      collision_monitor           |
                                             |                    |
                                           /cmd_vel                |
                                             |                    |
                                     adaptador G1 real             |
                                             | LocoClient.Move     |
                                             +-------------------> locomoción

sensores del G1 ---------------- datos fechados -----------------> adaptadores ROS
                                                              -> percepción/Nav2
                                                              -> tablero/registro
```

La separación física inicial es deliberada:

- La computadora de control del fabricante conserva equilibrio, marcha,
  límites y parada.
- La Jetson compartida del robot se mantiene sin instalaciones ni cambios
  persistentes durante el relevamiento. Si luego hace falta un adaptador a
  bordo, será mínimo, versionado y aprobado por quienes comparten el equipo.
- El runtime de desarrollo vive primero en una computadora Linux externa por
  Ethernet. Allí corren los adaptadores, Nav2, percepción y tablero.
- Los modelos lentos pueden seguir en un servidor externo; nunca entran en el
  lazo de equilibrio ni en la parada.

El enlace directo por cable reduce variabilidad y ofrece ancho de banda, pero
no es un mecanismo de seguridad. Toda orden debe vencer localmente si deja de
renovarse.

## Adaptador de movimiento que falta implementar

El contrato superior del repositorio se conserva:

```text
/g1/cmd_vel/{stand,navigation,alignment,manual,test}
                         |
                 mobility_authority
                         |
              /g1/cmd_vel/authorized
                         |
                 collision_monitor
                         |
                      /cmd_vel
                         |
                g1_loco_adapter
                         |
          Unitree LocoClient.Move(vx, vy, vyaw)
```

El nuevo `g1_loco_adapter` deberá cumplir estas condiciones antes de mover el
robot:

1. Confirmar que el servicio de locomoción de alto nivel está disponible y que
   el robot se encuentra en un modo compatible.
2. Tener un solo dueño de movimiento. La app, el control remoto y la autonomía
   no pueden competir ni suponerse compatibles sin una transición probada.
3. Limitar velocidades y aceleraciones con valores obtenidos en pruebas del
   ejemplar físico, no copiados de la simulación.
4. Rechazar órdenes vencidas. Si no llega una renovación dentro del plazo,
   enviar velocidad cero y `StopMove`.
5. En cancelación, pérdida de red, cierre normal o excepción, intentar una
   parada explícita. El operador y el control remoto siguen siendo la barrera
   independiente.
6. No usar inicialmente movimientos continuos de duración indefinida. El
   ejemplo Python oficial usa una duración acotada por defecto; nuestro
   adaptador renovará órdenes cortas y frescas.
7. Verificar el efecto físico con odometría y observación, no interpretar una
   respuesta RPC exitosa como prueba de que el cuerpo se movió o se detuvo.

La semántica exacta de modo, arbitraje con la app y recuperación de una sesión
interrumpida depende del firmware instalado. Debe probarse de nuevo después de
cada actualización.

## Datos reales y contratos internos

Durante el relevamiento del ejemplar físico se observaron topics de estado de
articulaciones, batería, IMU, control inalámbrico, odometría, SLAM y video. Son
un inventario útil de esa unidad, no un contrato público estable: antes de
integrar cada uno hay que registrar tipo, frecuencia, QoS, unidad, marco de
coordenadas y comportamiento cuando el productor se reinicia.

| Origen físico esperado | Adaptación interna | Uso |
|---|---|---|
| Estado de bajo nivel del G1 | `JointState`, IMU y diagnóstico propios de `/g1` | telemetría y comprobación, nunca comando |
| Estado de batería/BMS | mensaje tipado de batería | tablero, alertas y política de regreso |
| Video frontal del fabricante | un receptor y un decodificador, salida comprimida y cuadros fechados | tablero y percepción |
| Estimador/SLAM del fabricante | `Odometry` y `tf2`, sólo después de validar marcos y deriva | localización y navegación |
| Nube del LiDAR | `PointCloud2`, filtro corporal y proyección cuando corresponda | obstáculos, SLAM y Collision Monitor |
| `/cmd_vel` validado | adaptador de alto nivel `LocoClient` | locomoción física |

El repositorio que está instalado actualmente en el robot no expone en el
shell todos los tipos ROS 2 necesarios para interpretar algunos topics. Eso no
se resolverá instalando paquetes a ciegas en una Jetson compartida. Primero se
identificará la versión compatible y se construirá el adaptador fuera del
robot; cualquier componente a bordo tendrá un procedimiento reversible.

## Rendimiento de video y percepción

La dificultad de un tablero con video, detección y evidencia no está solamente
en ejecutar el modelo. El costo aparece al capturar, copiar, decodificar,
convertir, transportar, volver a codificar y conservar el mismo cuadro en
varios procesos. Si cada consumidor recibe video crudo, la cámara puede ocupar
el enlace y la memoria aunque el detector procese pocos cuadros.

El diseño actual ya tiene decisiones correctas:

- El detector conserva sólo el cuadro más reciente y descarta atraso.
- La inferencia corre en un worker separado y usa modo de inferencia.
- La búsqueda abierta y la lectura del reloj se ejecutan por evento, no sobre
  cada cuadro.
- Se publican demora, cuadros recibidos, procesados y descartados.
- La evidencia conserva la fecha del cuadro exacto que produjo la decisión.

Pero el camino de simulación no está listo para copiarse literalmente al video
físico:

- RGB crudo 640×480 más profundidad `float32` son aproximadamente 2,15 MB por
  cuadro. A 30 FPS equivalen a unos 64,5 MB/s, o 516 Mbit/s, antes de DDS,
  encabezados y copias.
- El tablero, el adaptador de detecciones y el localizador conservan historiales
  independientes. Con los límites actuales pueden superar 560 MiB sólo en
  cuadros crudos, sin contar modelos ni Python.
- El tablero vuelve a codificar JPEG y el navegador consulta cuadros. Ese
  mecanismo sirve a 3 FPS en simulación, pero no demuestra video fluido ni
  latencia acotada en el robot.
- El backend de detección compatible con CPU no está optimizado ni validado con
  TensorRT, Isaac ROS o DeepStream sobre el Orin real.

El camino físico objetivo será:

```text
stream comprimido del G1
          |
  receptor/decoder único
          |
 frame broker acotado y fechado
    |          |            |
 tablero   detector       evidencia
 comprimido latest-only   por evento
                             |
                    recorte o cuadro exacto
                             |
                       modelo remoto
```

La profundidad no se distribuirá a quien no la use. El registro de una foto al
detectar una clase será un consumidor asíncrono y acotado: recibe el cuadro
exacto, escribe el evento y no bloquea ni el video ni el control.

Antes de elegir aceleración se medirá latencia de captura a pantalla y de
captura a detección en percentiles p50 y p95, FPS realmente procesados, uso de
CPU/GPU, memoria, ancho de banda, temperatura y cuadros descartados. La
optimización se decide después de ubicar el cuello de botella.

## Riesgos abiertos antes de usar la demo en hardware

| Área | Qué puede fallar | Validación necesaria |
|---|---|---|
| Firmware y SDK | la interfaz o los modos cambian entre versiones | inventario inmutable y prueba contra la versión exacta |
| Sistema compartido | una instalación rompe servicios de otra persona | runtime externo primero; cambios a bordo versionados y reversibles |
| ROS 2 | el robot usa Foxy y la demo usa Jazzy | puente o adaptador de red probado; no mezclar entornos en el robot |
| Autoridad | app, control remoto y autonomía publican a la vez | protocolo explícito de adquisición, liberación y parada |
| Orden vencida | el robot conserva la última velocidad tras un corte | watchdog y ensayo real de desconexión |
| Odometría | una pose publicada puede derivar o usar otro marco | medir quietud, marcha, giro, retorno y reinicio |
| Tiempo | cámaras, LiDAR y pose no comparten reloj | comprobar timestamps, desfase y política de rechazo |
| Transformaciones | montaje y ejes no coinciden con Isaac | medir extrínsecos y revisar visualmente `tf2` |
| LiDAR | cuerpo y brazos aparecen como obstáculos; hay distorsión al caminar | filtro corporal, cobertura, frecuencia y compensación de movimiento |
| Cámara | codec, calibración, profundidad o FPS difieren | adaptador y ensayo de latencia/calidad sostenido |
| DDS y red | QoS incorrecto acumula datos o satura Ethernet | latest-only para video/sensores y entrega fiable sólo donde importa |
| Huella física | brazos o carga quedan fuera de la huella de Nav2 | huella dinámica y pruebas con cada postura aprobada |
| Manipulación | una orden de mano no confirma agarre | sensor o comprobación independiente antes de transportar |
| Energía | batería baja o apagado interrumpe escritura y control | alertas, umbral de regreso y cierre controlado de nuestros procesos |
| Privacidad | video o evidencias quedan guardados sin política | almacenamiento por evento, retención y acceso explícitos |
| Temperatura | inferencia sostenida reduce frecuencia por throttling | prueba de duración y métricas térmicas |

## Escalera de pruebas físicas

Cada escalón mantiene una referencia oficial sin modificaciones. Antes de una
prueba se declara una sola variable, la métrica que debería mejorar y el
criterio de aprobación. Se repite la medición y se complementa con inspección
visual.

### 0. Inventario sin cambios

Registrar versiones de firmware, SDK, sistema, ROS 2, interfaces de red,
sensores, topics, tipos y frecuencias. No actualizar ni instalar nada durante
el relevamiento.

### 1. Telemetría pasiva

Con el operador usando la app o el control remoto, leer batería, articulaciones,
IMU, odometría, LiDAR y video sin publicar comandos. Confirmar unidades,
timestamps, frecuencia y reinicios.

### 2. Tablero y percepción de sólo lectura

Mostrar video y telemetría desde el runtime externo. Ejecutar detección
latest-only y guardar una evidencia por evento. Hacer una prueba sostenida y
medir latencia p50/p95, FPS, pérdidas, memoria, red y temperatura.

### 3. Interfaz de locomoción sin desplazamiento

Consultar estado y modo mediante la API de alto nivel. Con arnés, área libre,
operador y parada disponibles, validar solamente la adquisición/liberación de
autoridad, orden cero, timeout y `StopMove`.

### 4. Movimiento mínimo en arnés

Cambiar una sola componente de velocidad por vez. Repetir quietud, avance corto,
detención y retorno a quietud; después lateral y giro. Medir demora de parada,
desplazamiento residual y diferencias entre orden y odometría.

### 5. Cadena Nav2 corta

Con límites físicos conservadores, probar un objetivo cercano, cancelación,
obstáculo, pérdida de comandos y pérdida de red. Collision Monitor y el
watchdog deben detener antes de ampliar distancia o velocidad.

### 6. Localización y sensores en movimiento

Validar mapa, deriva, `tf2`, nube, filtro corporal y huella dinámica durante
quietud, caminata y frenado. Una cifra buena sin una trayectoria visualmente
coherente no aprueba la etapa.

### 7. Misión integrada

Recién entonces conectar navegación, percepción, lectura remota y
manipulación. Cada capacidad conserva cancelación, plazo y resultado medido;
una falla deja al robot detenido y bajo autoridad del operador.

## Qué coincide con el fabricante y qué es adaptación propia

| Parte | Clasificación |
|---|---|
| `LocoClient` y comandos `Move`/`StopMove` | interfaz y flujo de ejemplo oficiales de Unitree |
| Control interno de equilibrio y marcha | componente propietario del fabricante; no afirmamos cómo está implementado |
| Conversión de `/cmd_vel` a renovaciones cortas de `LocoClient` | adaptación propia todavía por implementar y validar |
| `mobility_authority` y Collision Monitor antes del adaptador | arquitectura propia basada en patrones de ROS 2/Nav2 |
| WBC-AGILE en Isaac | referencia de simulación de NVIDIA, no locomoción aprobada para este G1 físico |
| Adaptadores de topics observados a contratos `/g1/*` | adaptación propia, dependiente de la versión del robot |
| Broker único de video y registro por evento | arquitectura propia a implementar, guiada por mediciones |

## Fuentes primarias

- [Ejemplo C++ oficial de locomoción de alto nivel del G1](https://github.com/unitreerobotics/unitree_sdk2/blob/main/example/g1/high_level/g1_loco_client_example.cpp)
- [Cliente C++ oficial `LocoClient`](https://github.com/unitreerobotics/unitree_sdk2/blob/main/include/unitree/robot/g1/loco/g1_loco_client.hpp)
- [Ejemplo Python oficial de locomoción de alto nivel del G1](https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/example/g1/high_level/g1_loco_client_example.py)
- [Implementación Python oficial de `LocoClient`](https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/unitree_sdk2py/g1/loco/g1_loco_client.py)
- [Repositorio oficial Unitree ROS 2](https://github.com/unitreerobotics/unitree_ros2)
- [QoS de ROS 2](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html)
- [Aplicación de referencia y medición de latencia de NVIDIA DeepStream](https://docs.nvidia.com/metropolis/deepstream/dev-guide/text/DS_ref_app_deepstream.html)
- [SDK oficial del LiDAR Unitree](https://github.com/unitreerobotics/unilidar_sdk2)
- [Integración oficial de Point-LIO para Unitree LiDAR](https://github.com/unitreerobotics/point_lio_unilidar)
