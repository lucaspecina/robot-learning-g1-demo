# Percepción: qué ve el robot y qué pasa con cada imagen

La imagen de **Lo que ve el robot** es la imagen RGB completa de su cámara
frontal. El tablero sólo agrega una franja negra con el número de cuadro y
comprime la imagen como JPEG para mostrarla en el navegador. No existe otra
vista más amplia que el robot use a escondidas.

## Flujo actual

```text
cámara frontal
      |
      v
RT-DETR local ----> cajas estándar `/g1/object_detections`
      |                         |
      |                         +--> cuadro exacto con cajas en el tablero
      v
adaptador de la demo
      |
      +--> reloj / botella / mesa roja o azul en `/g1/detections`
      |
      +--> sólo el recorte del reloj en `/g1/clock_crop/compressed`
                                |
                                v
                    modelo visual remoto, sólo al leer la hora
```

RT-DETR es un modelo liviano que encuentra objetos conocidos y devuelve un
rectángulo para cada uno. Corre a bordo y no necesita Internet. El modelo
remoto recibe únicamente un recorte cuando la tarea requiere entender un
detalle, por ejemplo leer los dígitos del reloj. Navegar y quedarse de pie no
dependen de ese modelo ni de la red externa.

## Memoria de imágenes

No se graba video infinito:

- El detector conserva sólo el cuadro más nuevo mientras trabaja. Si llegan
  más, descarta los viejos para no tomar decisiones atrasadas.
- El adaptador conserva en RAM hasta 60 cuadros identificados por su hora,
  aproximadamente 20 segundos a 3 cuadros por segundo. Esto permite unir una
  respuesta lenta con la imagen exacta que la produjo.
- El agente conserva sólo el último recorte del reloj y lo considera vencido
  después de 10 segundos.
- El tablero usa otro historial acotado de 60 cuadros para dibujar las cajas
  sobre el cuadro correcto. No escribe esas imágenes en disco.

Para investigar una falla concreta se podrá grabar una corrida acotada con las
herramientas de ROS 2. La grabación permanente no debe ser el modo normal del
robot físico por almacenamiento y privacidad.

## Qué puede observar Lucas

El tablero muestra:

1. el video vivo completo;
2. el último cuadro que realmente analizó el detector, con cajas y confianza;
3. el nombre del modelo, tiempo por análisis, cuadros procesados y descartados;
4. qué objeto produjo cada detección;
5. en el relato de la misión, el aviso anterior a una llamada remota y su
   respuesta.

Esto permite distinguir cuatro casos diferentes: el objeto no entró en la
cámara, entró pero el detector no lo reconoció, fue reconocido pero la tarea
no reaccionó, o falló el modelo remoto.

## Coincidencia con los flujos oficiales

| Parte | Referencia | Estado local |
|---|---|---|
| Cámara G1 simulada | Unitree: 7,6 mm, apertura 20 mm, 640×480, recta | mismos valores principales; montaje sobre nuestro `head_link` y 3 Hz son adaptaciones que deben medirse |
| Detección | NVIDIA Isaac ROS ofrece RT-DETR y publica cajas estándar de ROS 2 | mismo modelo conceptual y mismo tipo de mensaje |
| Búsqueda de categorías nuevas | NVIDIA ofrece Grounding DINO con una descripción escrita y recomienda usarlo de forma intercalada por su costo | prueba puntual correcta; todavía no integrado |
| Ejecución en la VM | Isaac ROS actual requiere una GPU más nueva que la T4 | backend compatible en CPU dentro de la Jetson simulada |
| Uso de modelo grande | procesar sólo cuando una tarea lo necesita | sólo se envía el recorte del reloj |

Antes de instalar el paquete acelerado en el G1 físico habrá que fijar una
combinación compatible entre la Jetson real, su versión de JetPack, ROS 2 e
Isaac ROS. “Es oficial” no garantiza que cualquier versión funcione con
cualquier Jetson.

## Experimentos de cámara del 29 de julio de 2026

| Cambio único | Resultado |
|---|---|
| cámara documentada como 20° abajo | el cálculo y la imagen probaron que miraba 20° arriba |
| cámara recta, lente anterior de 60° | mostró suelo y horizonte, pero dejó el reloj arriba del cuadro |
| cámara recta, lente oficial de unos 106° | reloj completo y centrado; 2/3 detecciones, todavía inestable |
| mesa con base maciza | entró completa en cuadro, pero visualmente parecía un cajón; 0/3 |
| base reemplazada por cuatro patas | ya parece mesa, pero a 320×240 sus partes son demasiado pequeñas; 0/3 |
| resolución oficial 640×480 | reloj 3/3, 0/3 falsos, RTF 0,23–0,24; la mesa entró completa pero RT-DETR quedó debajo del umbral |
| confianza cruda de RT-DETR sobre la mesa | `diningtable` fue la mejor clase, 0,574, con la caja correcta; también reveló una diferencia de nombre corregida |
| Grounding DINO pequeño, consulta “mesa roja / mesa azul” | mesa roja correcta a 0,618; 18,9 s en los dos CPU simulados, demasiado lento para ejecutarlo continuamente a bordo |

Una prueba numérica no cierra la cámara hasta que Lucas confirme también que
la imagen y las cajas tienen sentido.
