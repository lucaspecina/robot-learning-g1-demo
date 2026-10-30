# robot/

Recetas para trabajar con el G1 físico: grabar episodios, convertirlos al
formato de LeRobot y evaluar políticas sin mover el robot.

- [`unitree_lerobot_patches.md`](unitree_lerobot_patches.md) — los cambios
  que necesitó `unitree_lerobot` para nuestra configuración de una cámara.
- [`rtsp/rtsp_from_zmq.py`](rtsp/rtsp_from_zmq.py) — transmite por RTSP el
  video que ya reparte `teleimager-server` (JPEG por ZMQ), sin tomar la
  cámara: así convive con una política que la está usando. Probado el
  30-oct-2026 a 640×360 y 30 cuadros por segundo con menos del 20 % de CPU.
  Se corre en el robot con el Python del entorno `teleoperation` y `ffmpeg`.
- Teleoperación en simulación:
  [`docs/07_receta_teleoperacion.md`](../docs/07_receta_teleoperacion.md).
- Integración del sistema de la demo con el robot:
  [`g1/REAL_ROBOT_INTEGRATION.md`](../g1/REAL_ROBOT_INTEGRATION.md).

Las direcciones, credenciales y detalles de la unidad concreta van en `notas/`,
no acá.
