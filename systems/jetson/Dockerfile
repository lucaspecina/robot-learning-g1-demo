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
    ros-jazzy-nav2-msgs \
    ros-jazzy-vision-msgs \
    iproute2 \
    iputils-ping \
    curl \
    python3-pip \
    python3-requests \
    python3-numpy \
    python3-pil \
    python3-yaml \
    && rm -rf /var/lib/apt/lists/*

# La T4 de la VM no está soportada por Isaac ROS actual. Este RT-DETR en CPU es
# una implementación de compatibilidad: conserva la interfaz estándar que
# usará el paquete acelerado por NVIDIA en la Jetson física.
RUN python3 -m pip install --no-cache-dir --break-system-packages \
    torch torchvision --index-url https://download.pytorch.org/whl/cpu \
    && python3 -m pip install --no-cache-dir --break-system-packages \
    transformers==4.57.6 safetensors

ENV G1_OBJECT_DETECTOR_MODEL=PekingU/rtdetr_r50vd
ENV G1_OBJECT_DETECTOR_REVISION=df939e661d8c52e80608d1ec566561aabd25a4e7

# Guardar el modelo en la imagen hace que el robot pueda arrancar sin internet
# y evita que una actualización remota cambie silenciosamente los pesos.
RUN python3 -c "from transformers import AutoImageProcessor, RTDetrForObjectDetection; \
m='PekingU/rtdetr_r50vd'; r='df939e661d8c52e80608d1ec566561aabd25a4e7'; \
AutoImageProcessor.from_pretrained(m, revision=r, use_fast=True); \
RTDetrForObjectDetection.from_pretrained(m, revision=r)"

# El mapa se calcula a bordo con el paquete mantenido por Nav2. Este bloque va
# después del modelo pesado para que agregar navegación no obligue a descargar
# otra vez varios GB de percepción en cada experimento de infraestructura.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-slam-toolbox \
    ros-jazzy-nav2-map-server \
    && rm -rf /var/lib/apt/lists/*

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# El entrypoint de la imagen base ya hace `source /opt/ros/jazzy/setup.bash`
# para los comandos ejecutados via `docker exec ... bash -lc` no hace falta,
# pero dejamos un helper corto por comodidad:
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc

CMD ["sleep", "infinity"]
