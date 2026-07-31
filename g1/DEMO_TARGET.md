# Demo objetivo del G1

Esta es la misión de referencia del proyecto. La simulación debe reproducir las
entradas, sensores, computadoras, redes y límites que tendrá el robot real. No
se deben usar datos internos de Isaac en componentes que luego correrán en el
G1 físico.

## Misión

1. El G1 comienza en un lugar seguro y guarda su posición como `home`.
2. Una orden de voz lo activa y le pide consultar la hora y traer un objeto.
3. El robot se ubica en el mapa y encuentra el reloj.
4. Se coloca frente al reloj y lee la hora con la cámara.
5. Si es antes de las 12:00 del mediodía, elige la mesa A, roja.
6. Si son las 12:00 o más, elige la mesa B, azul.
7. Las mesas no figuran por nombre ni posición en el mapa: debe buscar la mesa
   elegida con sus sensores.
8. Se acerca en dos etapas: navegación general y alineación fina.
9. En el tramo actual, aparece un bulto de masa conocida entre las muñecas;
   más adelante, las manos deben agarrar físicamente el objeto de la mesa.
10. Adopta una postura estable de transporte y regresa a `home`.

Las personas de la versión anterior quedan fuera de alcance hasta completar
esta misión.

## Qué sabe el robot y qué debe descubrir

- El mapa guardado contiene paredes, espacio transitable y obstáculos fijos.
- El LiDAR aporta geometría y ubicación; no reconoce por sí solo un reloj ni
  distingue una mesa roja de una azul.
- La cámara reconoce el reloj, lee la hora e identifica el color de las mesas.
- La cámara de profundidad permite convertir una detección visual en distancia
  y posición dentro del mapa.
- `home` se registra al comenzar cada misión; no es una coordenada fija.

Durante las primeras pruebas se permite conocer la posición del reloj para
aislar navegación y cámara. En la versión completa, el robot debe encontrarlo
o usar una anotación obtenida durante la preparación real del lugar. Cualquier
atajo de simulación debe estar marcado y tener un reemplazo previsto.

## Cómo se ejecutará la misión

El modelo propone un plan inicial usando solamente el catálogo declarado. La
Jetson valida el plan y ejecuta una capacidad por vez. Después de cada
capacidad recoge el resultado medido y, si la decisión es visual, una imagen
puntual. El modelo puede conservar o modificar sólo los pasos pendientes.

Si una capacidad deja de progresar, la Jetson la cancela localmente, devuelve
la movilidad a `STAND` y recién entonces consulta al modelo. La seguridad y el
equilibrio no dependen de la red.

El contrato completo y el orden de implementación están en
[`AGENT_EXECUTION_PLAN.md`](AGENT_EXECUTION_PLAN.md). Nav2, SLAM, video
continuo y una VLA de cuerpo completo quedan fuera de este tramo.

Después de aprobar visualmente este tramo se ejecutará la auditoría de ayudas
y el camino de LiDAR a Nav2 definidos en
[`DEPLOYMENT_READINESS_PLAN.md`](DEPLOYMENT_READINESS_PLAN.md). Una misión con
carga anexada puede aprobar locomoción con peso, pero no cuenta como agarre ni
como ensayo de despliegue.

## Preparación del lugar real

El mapeo no se repite obligatoriamente antes de cada misión:

1. En la instalación inicial, recorrer el lugar usando LiDAR y profundidad.
2. Construir y guardar el mapa a partir de sensores.
3. En cada encendido, cargar ese mapa y localizar al robot dentro de él.
4. Actualizar o reconstruir el mapa cuando el entorno cambie materialmente.

Primero habrá un inicio fijo. Después se elegirá al azar entre tres posiciones
seguras y verificadas. No se usará una posición arbitraria hasta que
localización, obstáculos y recuperación estén medidos.

## Escalera de validación

1. Mantenerse quieto, caminar y frenar.
2. Llegar al reloj conocido, orientarse y verlo centrado.
3. Leer correctamente horas anteriores y posteriores a las 12:00.
4. Guardar `home`, alejarse y regresar sin carga.
5. Construir/cargar el mapa y localizarse usando sensores simulados.
6. Buscar una mesa de color cuya posición no recibe el agente.
7. Navegar hasta una posición cercana y alinearse a `2–3 cm`.
8. Agregar una carga física simulada después de alinearse, sin presentarla
   como agarre, para desbloquear y medir el regreso cargado.
9. Reemplazar esa adaptación por manos, contacto y agarre físico repetible.
10. Transportar cargas crecientes y regresar a `home`.
11. Activar la misma misión por voz.
12. Repetir con latencia, ancho de banda limitado y cortes de red.

Una capacidad nueva pasa de etapa después de una comprobación rápida, una
inspección visual y tres repeticiones completas. Las regresiones físicas de
quietud, caminata y frenado son obligatorias cuando cambia física o locomoción,
no cuando cambia solamente una frase, un prompt o una regla de decisión.
