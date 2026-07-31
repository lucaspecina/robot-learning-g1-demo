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

La misión puede pedirlo en caliente mediante `attach_payload`: PhysX modifica
la masa absoluta de ambas muñecas, la relee y publica la confirmación. Un bulto
naranja sin colisión sigue visualmente el punto medio entre ellas. Ese dibujo
no aporta otra masa; separar la representación de la carga evita contarla dos
veces. Es una adaptación propia y temporal porque el cuerpo AGILE activo no
incluye los dedos Dex3. El flujo final de NVIDIA la reemplazará por el objeto
dinámico unido a la mano después de confirmar un agarre.

Referencias oficiales consultadas:

- Isaac Sim documenta uniones fijas físicamente simuladas entre cuerpos:
  <https://docs.isaacsim.omniverse.nvidia.com/latest/robot_setup/assemble_robots.html>
- El flujo de pick-and-place separa estimación de pose, aproximación, agarre,
  retracción y objeto sujeto durante el transporte:
  <https://nvidia-isaac-ros.github.io/concepts/manipulation/pick_and_place.html>
- El flujo G1 de NVIDIA usa manos Dex3 y teleoperación para reunir las
  demostraciones del agarre real:
  <https://nvidia-isaac-ros.github.io/reference_workflows/isaac_for_physical_ai/tutorials/tutorials.html>

Comandos de prueba, con el robot ya suelto y en `STAND`:

```bash
bash run_demo.sh check stand 0.5
bash run_demo.sh check walk 0.5
bash run_demo.sh payload detach
```

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

## Resultado medido del 31-jul-2026

La masa se releyó en las dos muñecas antes de cada corrida.

- Con `0,5 kg`, tres caminatas avanzaron `1,77–2,08 m`, se desviaron
  `12–16 cm` y frenaron en `0–3 cm`, sin caídas. Tres navegaciones relativas
  terminaron a `9–10 cm` del objetivo.
- Con `1,0 kg`, tres caminatas avanzaron `2,11–2,19 m`, se desviaron
  `10–17 cm` y frenaron en `1–2 cm`, sin caídas. Una navegación terminó a
  `9 cm` del objetivo.
- Sin embargo, con `1,0 kg` el hombro derecho sostuvo `-12,5°` frente a
  `-14,3°` pedidos: `1,9°` de error contra `1,7°` permitidos. El fallo se
  mantuvo durante 40 s y no se relajó la tolerancia.

Por eso la misión usa `0,5 kg`, que coincide con el objeto de la escena y
aprobó el sistema completo. Un kilogramo queda como límite de locomoción
prometedor, pero no como carga aprobada de cuerpo y brazos. No se continúa a
`2 kg` hasta mejorar la postura o el controlador de brazos con una métrica
declarada.

El 31-jul la carga aprobada se integró en la misión completa: se aplicaron
`0,25 kg` en cada muñeca, los brazos llegaron a transporte con `0,0287 rad` de
error máximo y el robot volvió al inicio sin caer. La primera llegada expuso
un margen excesivo del navegador y quedó a `18,8 cm`; después de retirar sólo
ese margen, el verificador ida/regreso con la misma carga volvió a `8,3 cm`,
`4,5°` y `0,764 m` de altura. La inspección visual completa de Lucas sigue
siendo obligatoria antes de cerrar la demo.

La inspección visual del 31-jul rechazó la postura aunque las articulaciones
llegaron al objetivo numérico: el bulto quedó superpuesto con la zona de la
pelvis y los brazos no parecían sostener un objeto transportable. Esto no
invalida la medición de locomoción con masa repartida en las muñecas, pero sí
invalida llamar “aprobada” a la pose de transporte o a su representación. Los
ángulos son una adaptación nuestra, no una pose oficial de NVIDIA. Antes de
otra misión integral se probará la nueva pose con el robot congelado, se
medirá la posición de ambas muñecas y Lucas deberá aprobarla visualmente.

El banco aún publica `/g1/reset` sin confirmación correlacionada. Una de tres
navegaciones comenzó a `32 cm` del origen esperado, aunque el objetivo relativo
y la llegada fueron válidos. Antes de usar el reinicio para comparar posiciones
absolutas se agregará una respuesta explícita del robot.

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
