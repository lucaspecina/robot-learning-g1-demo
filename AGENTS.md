# Reglas de trabajo del proyecto G1

## Fuentes antes de diseñar

Antes de diseñar o cambiar una parte del sistema:

1. Consultar primero la documentación oficial de los productos involucrados
   (NVIDIA Isaac Sim/Isaac Lab/AGILE, Unitree, ROS 2/Nav2, etc.).
2. Revisar después implementaciones y prácticas de comunidades técnicas
   confiables, priorizando repositorios oficiales, papers y laboratorios con
   resultados reproducibles.
3. Repetir esa consulta cada vez que aparezca una duda, una anomalía o una
   decisión nueva. Una investigación anterior no reemplaza verificar la
   recomendación específica del problema actual.
4. Registrar qué parte coincide con el flujo oficial y qué parte es una
   adaptación propia. Nunca describir una integración como “oficial” si se
   modificaron versiones, configuración, cuerpo, entradas o controladores.
5. Si nos apartamos de la práctica recomendada, hacerlo solamente por una
   necesidad medida y dejar documentados la evidencia, el costo y el camino
   para revertirlo.

## Disciplina experimental

- Cambiar una sola variable física por experimento.
- Declarar antes la métrica que debería mejorar y el criterio de aprobación.
- Repetir las mediciones; una sola ejecución no prueba una tendencia.
- Después de cada cambio físico o de locomoción, probar quietud, caminata y
  frenado antes de continuar.
- Complementar las métricas con inspección visual. Un resultado numérico no
  valida por sí solo que el movimiento tenga el sentido esperado.
- Si un cambio no mejora la métrica declarada, revertirlo.
- Mantener una referencia oficial sin modificaciones y compararla contra la
  integración de la demo.

## Convenciones

- Identificadores, archivos, módulos y funciones: en inglés.
- Comentarios, docstrings y mensajes para el usuario: en español.
- Los comentarios explican por qué existe una decisión, no repiten qué hace
  el código.
- Explicar robótica en lenguaje humano; definir cualquier término técnico la
  primera vez que se use.
