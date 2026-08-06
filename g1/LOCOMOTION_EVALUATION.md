# Evaluación de locomoción y deriva

Fecha de la campaña actual: 2026-07-28.

## Decisión que buscamos tomar

La pregunta no es qué ajuste “se ve mejor”, sino qué conjunto puede servir de
base para la demo y luego transferirse al G1 real:

1. el cuerpo exacto del robot;
2. el programa aprendido que mueve las piernas;
3. las mediciones que recibe;
4. la forma en que se representan los motores;
5. las frecuencias de física y control.

No se integrará un candidato porque logre caminar una vez. Tiene que superar,
como mínimo, pruebas repetidas de permanecer quieto y de caminar y frenar.

## Referencia encontrada en la comunidad

NVIDIA publicó
[WBC-AGILE](https://github.com/nvidia-isaac/WBC-AGILE), un conjunto validado
para el G1 completo que usa nuestras versiones exactas: Isaac Sim 5.1 e
IsaacLab 2.3.2. Incluye un programa ya entrenado que:

- conoce las 29 articulaciones del cuerpo;
- controla solamente las 12 articulaciones de las piernas;
- deja la cintura y los brazos bajo un control separado;
- recibe una orden de velocidad y altura;
- está preparado para ejecutarse también en MuJoCo y en el robot real.

IsaacLab usa el mismo patrón en su
[ejemplo oficial de agarrar y mover objetos con el G1](https://github.com/isaac-sim/IsaacLab/blob/main/source/isaaclab_tasks/isaaclab_tasks/manager_based/locomanipulation/pick_place/locomanipulation_g1_env_cfg.py):
un programa para las piernas y otro control separado para la parte superior.

Esto coincide con la separación que necesita nuestra demo. No coincide con
la combinación anterior, que puso un programa preparado para el G1
simplificado de 12 articulaciones dentro de nuestro cuerpo personalizado de
29 articulaciones.

## Método

Cada corrida:

- usa tiempo simulado, no tiempo de pared;
- conserva la física oficial sin ajustes;
- desactiva empujones y cambios aleatorios de fricción o masa;
- guarda la posición y velocidad en cada ciclo;
- usa un solo robot;
- se repite tres veces antes de aceptar una tendencia.

Los archivos que reproducen la campaña son:

- `tools/wbc_agile_stand.yaml`;
- `tools/wbc_agile_walk_stop.yaml`;
- `tools/run_wbc_agile_eval.sh`;
- `tools/analyze_wbc_agile_eval.py`.

## Resultados confirmados del controlador anterior

| Prueba | Resultado |
|---|---:|
| Isaac, cuerpo personalizado de 29 articulaciones | 9,0 cm/s |
| Isaac, cuerpo convertido de 12 articulaciones | 7,6 cm/s |
| MuJoCo oficial con el mismo archivo aprendido | 2,4 cm/s |

La trayectoria en Isaac es recta y sostenida. Por eso no parece solamente
ruido numérico.

También quedaron medidos estos descartes:

- Cambiar la fricción del piso de 0,5 a 1,0 no cambió la deriva.
- Agregar la fricción y la inercia interna de las articulaciones del modelo
  oficial no cambió la deriva.
- Reiniciar la memoria interna del programa no cambió la deriva.
- El cuerpo de 12 y el de 29 articulaciones presentan el mismo tipo de error.
- Las 47 entradas del programa anterior coinciden con el despliegue de
  Unitree.
- Los límites nominales de fuerza y velocidad **no hacen caer al robot** en
  la versión verificada actual. Quietud y caminata dieron la misma magnitud
  que con límites amplios. La afirmación anterior de que necesitaba
  `300 N·m / 100 rad/s` quedó refutada por medición.
- Cambiar solamente la distancia de detección del contacto del pie de
  `0,0002 m` a `0,001 m` no cambió ni la quietud ni la caminata.

Por lo tanto, seguir ajustando fricción, contacto o fuerza sin una nueva
evidencia no está justificado.

## Base oficial de NVIDIA, sin modificar

En la prueba oficial de permanecer quieto, WBC-AGILE mueve la parte superior
del cuerpo al azar para comprobar que las piernas toleren perturbaciones. No
es equivalente a nuestra espera normal con los brazos en una pose fija.

| Corrida | Desplazamiento en 60 s | Altura mínima |
|---|---:|---:|
| 1 | 51,8 cm | 70,0 cm |
| 2 | 17,6 cm | 69,9 cm |
| 3 | 8,3 cm | 69,3 cm |
| **Promedio** | **25,9 cm** | — |

Las tres corridas se mantuvieron de pie. El promedio equivale a
`0,43 cm/s`, frente a `9,0 cm/s` del controlador anterior: aproximadamente
veinte veces menos deriva incluso mientras los brazos hacen movimientos
grandes.

La variación entre corridas también es importante. Una única lectura habría
dado una conclusión engañosa.

## Caminar y frenar con la parte superior aleatoria

La orden fue:

- 0–5 s: quieto;
- 5–15 s: avanzar a `0,3 m/s`;
- 15–30 s: volver a cero.

| Corrida | Velocidad frontal media al caminar | Desplazamiento durante los últimos 10 s parado |
|---|---:|---:|
| 1 | 0,246 m/s | 29,7 cm |
| 2 | 0,271 m/s | 19,6 cm |
| 3 | 0,280 m/s | 2,5 cm |

Camina sin caerse y sigue razonablemente la orden, pero el frenado todavía
varía demasiado. Durante la espera final, las articulaciones de la parte
superior recorren en promedio unos `2 rad` —más de 110 grados— porque esa
prueba oficial las mueve deliberadamente.

## Experimento de una variable: parte superior quieta

Se fijaron cintura y brazos en su pose neutral. No se cambió la física, el
cuerpo, el programa de piernas ni sus entradas.

| Corrida | Desplazamiento después de los primeros 5 s |
|---|---:|
| 1 | 1,1 cm en 55 s |
| 2 | 2,1 cm en 55 s |
| 3 | 1,3 cm en 55 s |
| **Promedio** | **1,5 cm en 55 s** |

Las tres corridas se mantuvieron de pie. En caminar y frenar, la velocidad
frontal media fue `0,268 m/s` en las tres repeticiones para una orden de
`0,3 m/s`. Durante los últimos 10 s quieto se desplazó `4,6`, `3,8` y
`7,3 mm`.

La reducción respecto de los `9 cm/s` anteriores es de aproximadamente
trescientas veces. La puerta para integrar quedó aprobada.

## Integración dentro de la demo

No se copió solamente el archivo `.pt`. `g1_robot.py` usa ahora como conjunto:

- el G1 oficial de 29 articulaciones;
- la representación de motores de AGILE, incluido su retraso;
- la frecuencia oficial de 200 Hz de física y 50 Hz de control;
- el descriptor oficial de 80 entradas y 12 salidas;
- la policy recurrente desplegable de NVIDIA.

La dependencia está fijada al commit
`7259792cf10803aab814d101134d493d24c8f22f`. El arranque se niega a usar otra
versión sin que se la vuelva a verificar.

Pruebas dentro del proceso real de la demo:

| Prueba | Resultado |
|---|---:|
| Quieto, brazos neutrales | 1,25 cm en 55 s, sin caída |
| Caminar recto, brazos neutrales | 0,264 m/s para orden 0,3 m/s |
| Rumbo en esa caminata | -4,7° y -0,88 m lateral en 25 s |
| Cámara y escena | publican correctamente |
| Cámara a 3 Hz simulados | verificada; ya no repite cuadros a 50 Hz |
| Arranque congelado | 74,4 cm, cámara y ROS activos |

La traslación natural de la policy bajó mucho, pero eso no autoriza a declarar
resuelta la quietud del sistema completo. `stand_hold` debe sostener un anclaje
y aprobar repetidamente; la navegación no debe usarse para esconder este
problema.

## Comparación directa con el despliegue oficial

La documentación de NVIDIA distingue dos archivos: el `checkpoint` sirve para
entrenamiento y evaluación por lotes; el TorchScript junto con su YAML es el
archivo listo para desplegar. La demo usa el segundo.

Primero se comparó nuestro adaptador con las clases oficiales
`ObservationProcessor` y `ActionProcessor`, usando el mismo TorchScript y ocho
estados deterministas:

| Contrato comparado | Resultado |
|---|---:|
| Nombres y orden de las 29 articulaciones observadas | iguales |
| Nombres y orden de las 12 articulaciones controladas | iguales |
| 80 entradas de la policy | error máximo `0.0` |
| 12 salidas crudas de la policy | error máximo `0.0` |
| 12 objetivos enviados a los motores | error máximo `0.0` |

La auditoría encontró antes una diferencia real: NVIDIA conserva la salida
anterior limitada a `±10` como memoria de la próxima decisión, pero limita a
`±6` lo enviado al motor. Nosotros usábamos `±6` para ambas cosas. Se corrigió
y recién entonces la comparación dio cero. El error sólo aparece ante picos
mayores que seis, por lo que no explica por sí solo el desvío ordinario.

El evaluador oficial no puede alimentar directamente el TorchScript recurrente
con una sola réplica: agrega una dimensión y luego intenta abrirlo como si
fuera un `checkpoint`. Para probar la física oficial se hizo una adaptación
de forma solamente: `[1, 80] → [80]` antes de la policy y `[12] → [1, 12]`
después. No se modificaron el cuerpo, la física, la policy, las mediciones ni
los comandos.

Comparación con la misma secuencia: 5 s quieto, 10 s a `0,3 m/s` y 15 s
detenido:

| Entorno | Corridas | Avance | Costado | Ángulo de trayectoria |
|---|---:|---:|---:|---:|
| MuJoCo, adaptador oficial y export oficial | 3 | 2,59 m | -8,2 cm | 1,82° |
| Isaac oficial, export oficial | 3 | 1,82–2,07 m | -9,3 a -65,0 cm | 2,6–19,7° |
| Demo Isaac, integración real | 3 | 2,47–2,56 m | -23 a -35 cm | 5,1–7,8° |

En la demo el frenado fue bueno (`5–8 cm`) y no hubo caídas, pero las tres
caminatas fallaron el límite de `5°`. La quietud con `stand_hold` aprobó dos
veces y falló una: el máximo fue `0`, `5` y `17 cm`. Por lo tanto, el cableado
de la policy está verificado, pero el comportamiento físico completo todavía
no está aprobado.

La escena oficial de Isaac deja libre la parte superior y en estas corridas
los brazos se movieron varios radianes. No es idéntica a la demo, que sostiene
los brazos, pero sirve para responder la sospecha principal: incluso el flujo
oficial se desvía mucho y varía entre corridas. Nuestra integración no muestra
la firma de ejes intercambiados ni un resultado peor que la referencia.

La conclusión de diseño cambia el lugar donde se exige precisión:

- La locomoción de piernas debe mantenerse de pie, responder en la dirección
  pedida y frenar. No garantiza por sí sola una trayectoria global recta.
- Navegación debe corregir rumbo y posición mientras es la única dueña del
  movimiento. Esa corrección es parte normal del sistema, no un parche.
- `stand_hold` debe mantener el anclaje cuando nadie navega. El fallo medido de
  `17 cm` motivó el experimento separado que aparece debajo.
- La alineación junto a mesa o persona debe tener su prueba propia y una
  tolerancia mucho menor que la espera libre.

El primer A/B de `stand_hold` cambió sólo la velocidad máxima de corrección de
`0,45` a `0,15 m/s`. El límite anterior permitía perseguir el anclaje casi a
velocidad de marcha.

| Corrida quieto | Máximo respecto del anclaje |
|---|---:|
| 1 | 1 cm |
| 2 | <1 cm |
| 3 | <1 cm |

Se conserva el cambio. La caminata posterior avanzó `2,56 m`, se desvió `4,8°`,
no cayó y frenó en `5 cm`.

## Poses de brazos

Las poses `listo` y `transporte` son objetivos articulares escritos por
nosotros. No localizan el objeto ni cierran dedos.

| Prueba | Resultado |
|---|---:|
| Transición a `listo` | hasta 17,6 cm de reajuste, sin caída |
| `listo` ya estable | 5,6 mm en 20 s |
| Transición a `transporte` | hasta 18,8 cm de reajuste, sin caída |
| `transporte` ya estable | 4,1 mm en 20 s |

La misión debe cambiar de pose antes de la alineación precisa, esperar que el
cuerpo se estabilice y recién después acercarse o agarrar.

### Caminar con `transporte`

La secuencia correcta fue: 10 s para adoptar la pose, 20 s de orden de
caminata y 10 s para frenar. Se repitió tres veces.

| Corrida | Avance analizado | Desvío lateral | Cambio de rumbo |
|---|---:|---:|---:|
| 1 | 4,52 m | -1,04 m | -10,9° |
| 2 | 4,54 m | -1,32 m | -14,8° |
| 3 | 4,51 m | -0,88 m | -8,3° |

No hubo caídas y el frenado fue estable, pero la pose actual no aprueba el
criterio de caminar recto sin corrección. AGILE deja la parte superior bajo
control separado, pero esta policy fue entrenada para caminar con la parte
superior neutral. Observar los brazos no garantiza que cualquier postura
quede dentro de lo aprendido.

La próxima decisión debe medirse visualmente:

1. probar una pose de transporte más cercana a neutral;
2. repetir quietud, caminata, giro y frenado;
3. agregar carga mediante `PAYLOAD_TEST_PLAN.md`;
4. si ninguna pose útil conserva el rumbo, continuar el entrenamiento de la
   locomoción con poses y cargas variables.

La navegación puede corregir errores pequeños de rumbo, pero no se aceptará
como forma de ocultar una pose incompatible con la locomoción.

## Decisión

WBC-AGILE reemplaza a la locomoción anterior como base de la demo. El contrato
de despliegue está integrado y verificado; no hay evidencia de articulaciones
intercambiadas ni de entradas desordenadas.

No está verificado todavía que nuestra ejecución física reproduzca la
referencia oficial con precisión suficiente. Hasta terminar la comparación
Isaac contra Isaac, no corresponde atribuir la diferencia a contactos,
solucionador, control de brazos o navegación.

Quedan como trabajos separados:

- aislar la primera diferencia física contra la escena oficial de Isaac;
- lograr que quietud y rumbo aprueben tres repeticiones;
- inspección visual de las pruebas;
- diseño de una pose de transporte compatible;
- escalera de cargas;
- alineación final activa;
- agarre real con localización de mano y control de dedos.
