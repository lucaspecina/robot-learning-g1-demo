# La "Jetson" simulada: la computadora de a bordo del robot.
#
# Contenedor con ROS 2 Jazzy donde van a vivir los nodos que en el robot real
# corren en la Jetson: driver-consumidores, navegacion, percepcion, skills,
# watchdogs. Se lanza con limites de recursos (--cpus, --memory) para imitar
# el presupuesto de una Orin.
#
# Decision importante: usa CycloneDDS como motor de mensajeria (en vez del
# FastDDS por defecto). Motivo: FastDDS detecta "misma maquina" entre el host
# y el contenedor y fuerza un atajo de memoria compartida que no funciona a
# traves de la frontera del contenedor — y falla en silencio. CycloneDDS no
# tiene ese atajo: todo va por UDP, que cruza la frontera sin problema y ademas
# es lo realista (en el robot real este enlace es un cable ethernet).
# Los dos motores se entienden entre si porque DDS es un protocolo estandar.
FROM ros:jazzy-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-rmw-cyclonedds-cpp \
    iproute2 \
    iputils-ping \
    curl \
    python3-requests \
    && rm -rf /var/lib/apt/lists/*

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# El entrypoint de la imagen base ya hace `source /opt/ros/jazzy/setup.bash`
# para los comandos ejecutados via `docker exec ... bash -lc` no hace falta,
# pero dejamos un helper corto por comodidad:
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc

CMD ["sleep", "infinity"]
