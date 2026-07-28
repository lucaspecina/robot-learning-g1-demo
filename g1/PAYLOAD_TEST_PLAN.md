# Pruebas de transporte con carga

Este protocolo decide con mediciones si la locomoción de NVIDIA AGILE puede
transportar la carga de la demo y si hace falta continuar su entrenamiento.

## Condición previa

Antes de agregar peso, el robot debe aprobar `stand` y `walk` sin carga con la
pose de brazos `transporte`. El programa de prueba debe confirmar en el log:

- qué piezas recibieron la masa;
- cuánta masa recibió cada una;
- masa total antes y después del cambio.

El cuerpo de AGILE desactiva las manos con dedos para acelerar la simulación.
Por eso la carga de locomoción se aplica a los dos últimos cuerpos físicos
disponibles, las muñecas `wrist_yaw_link`. Si no encuentra exactamente dos
puntos de mano o muñeca, la prueba falla. Nunca se acepta una corrida que pidió
carga pero no demuestra que la aplicó.

Esto mide equilibrio con peso. No demuestra que exista un agarre ni reproduce
todavía la distancia exacta entre la muñeca y el centro del objeto.

## Escalera obligatoria

Cada nivel se prueba por separado y sólo se avanza si el anterior es estable:

1. 0 kg, para establecer la referencia con pose `transporte`;
2. 0,5 kg;
3. 1,0 kg;
4. 2,0 kg;
5. 3,0 kg, únicamente si 2,0 kg aprobó.

Primero se reparte la carga por igual entre ambas manos. Después se repite el
mayor peso aprobado en una sola mano, porque la carga lateral es más exigente
y se parece a un agarre imperfecto.

## Pruebas en cada nivel

El orden no se salta:

1. permanecer quieto 60 segundos simulados;
2. caminar en línea recta;
3. frenar y permanecer quieto;
4. girar hacia ambos lados;
5. recorrer un trayecto de navegación;
6. mantener la posición final cerca de la mesa.

Cada prueba se repite tres veces. Se guardan el caso medio y el peor, no sólo
el promedio.

## Mediciones

- caídas;
- desplazamiento mientras debería estar quieto;
- error lateral y de rumbo al caminar;
- distancia necesaria para frenar;
- error final de navegación;
- altura e inclinación del torso;
- velocidad y esfuerzo de las articulaciones;
- deslizamiento o pérdida del objeto cuando exista un agarre físico.

## Decisión

La policy actual se conserva si aprueba el peso necesario para la botella con
margen y sin degradar la precisión requerida junto a la mesa o la persona.

Si falla, el orden de respuesta es:

1. acercar la carga al torso y mejorar la pose `transporte`;
2. reducir aceleración, velocidad y brusquedad de giro mientras carga;
3. verificar que la masa y su ubicación representen al objeto real;
4. continuar el entrenamiento de la policy de locomoción con pesos, poses de
   brazos y cargas laterales variables.

Continuar el entrenamiento es el último paso porque cambia una pieza costosa.
La navegación nunca se usa para esconder una locomoción incapaz de sostener la
carga.
