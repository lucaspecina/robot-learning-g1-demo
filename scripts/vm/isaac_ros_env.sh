#!/usr/bin/env bash
# Entorno para correr Isaac Sim 5.1 con su puente ROS 2 (libs Jazzy internas).
# Uso:  source isaac_ros_env.sh && cd /opt/IsaacSim && ./python.sh <script>
#
# Nota: NO sourcear el ROS del sistema en la misma shell (python de kit es 3.11;
# el rclpy del sistema es de python 3.12 — el puente usa sus libs internas).
#
# Vivía sólo en el disco de la VM y no estaba versionado. Rescatado el
# 10-ago-2026 para poder reconstruir el entorno en una máquina nueva.
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH=/opt/IsaacSim/exts/isaacsim.ros2.bridge/jazzy/lib
# rclpy interno importable ANTES de arrancar SimulationApp (repos que lo importan arriba)
export PYTHONPATH=${PYTHONPATH:-}:/opt/IsaacSim/exts/isaacsim.ros2.bridge/jazzy/rclpy

# OJO: en la VM original esta línea apunta a un archivo que YA NO EXISTE
# (`~/go2-lab/sistemas/fastdds_solo_udp.xml`, con nombre viejo en castellano).
# El archivo vigente del repo es `systems/fastdds_udp_only.xml`. Se deja
# comentado porque el diseño actual usa CycloneDDS, no FastDDS; descomentar
# sólo si se vuelve a necesitar y apuntando a la ruta correcta.
# export FASTDDS_DEFAULT_PROFILES_FILE=$HOME/go2-lab/robot-learning-g1-demo/systems/fastdds_udp_only.xml
