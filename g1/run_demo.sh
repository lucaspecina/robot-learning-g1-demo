#!/usr/bin/env bash
# Levanta el sistema completo de la demo, en orden, y deja todo listo para mirar.
#
# Uso (en la VM):
#   bash run_demo.sh up          arranca robot + skills + agente
#   G1_OPERATION_PROFILE=deployment_rehearsal bash run_demo.sh up
#                               arranca sin coordenadas ni carga mágicas
#   bash run_demo.sh check [cual] LA ESCALERA: stand | walk | goto | clock | home
#   bash run_demo.sh check obstacle exige un rodeo físico con Nav2
#   bash run_demo.sh clock      va al reloj conocido y termina mirándolo
#   bash run_demo.sh read-clock lee el recorte vivo mediante el servidor
#   bash run_demo.sh table red  mira una mesa desde una pose sólo de prueba
#   bash run_demo.sh align-table red  alinea desde una mesa ya visible
#   bash run_demo.sh pose NOMBRE  mueve brazos: reposo | listo | transporte
#   bash run_demo.sh payload attach KG  agrega carga física; detach la retira
#   bash run_demo.sh mission     le da la misión completa al planificador
#   bash run_demo.sh layers      reinicia sólo las capas de la Jetson
#   bash run_demo.sh status      como viene todo
#   bash run_demo.sh down        apaga las capas de arriba (no el robot)
#
# El orden correcto es: up -> check all -> recién ahí mission. Un plan correcto
# no significa nada si el robot no se sostiene de pie.
#
# EL TABLERO VA APARTE, a proposito: es un observador, no parte del robot.
# Se prende una vez (tablero on) y se queda vivo mientras el robot nace, se
# cae y vuelve a arrancar — que es justo cuando uno mas necesita mirarlo.
#
# Para mirarlo desde tu maquina, dos ventanas:
#   - el tablero:  ssh -L 8080:localhost:8080 lucas@<IP>  -> http://localhost:8080
#   - el robot:    cliente de Isaac apuntando a la IP de la VM
#
# En Isaac, mantener la vista en Perspective: rueda para zoom, boton central
# para desplazar, Alt + boton izquierdo para girar y F para centrar el objeto
# seleccionado. El seguimiento automático queda apagado para no deshacer los
# movimientos del operador. No mover /head_cam: es la cámara física del robot.
set -uo pipefail

HOST_DEMO_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
D=/workspace/g1          # ruta dentro del contenedor jetson
ROS="source /opt/ros/jazzy/setup.bash"
OPERATION_PROFILE_FILE=/tmp/g1_operation_profile

resolve_operation_profile() {
    local profile="${G1_OPERATION_PROFILE:-}"
    if [ -z "$profile" ] && [ -r "$OPERATION_PROFILE_FILE" ]; then
        profile=$(tr -d '\r\n' < "$OPERATION_PROFILE_FILE")
    fi
    profile="${profile:-simulation_demo}"
    case "$profile" in
        simulation_demo|deployment_rehearsal) printf '%s' "$profile" ;;
        *)
            echo "ERROR: perfil inválido: $profile" >&2
            echo "usar simulation_demo o deployment_rehearsal" >&2
            return 2
            ;;
    esac
}

OPERATION_PROFILE=$(resolve_operation_profile) || exit $?

# Dos operaciones de banco simultáneas invalidan cualquier medición y pueden
# duplicar todos los nodos. `status` queda libre para poder observar mientras
# una prueba corre; las operaciones que cambian estado son exclusivas.
case "${1:-up}" in
status) ;;
*)
    exec 8>/tmp/g1_demo_operation.lock
    if ! flock -n 8; then
        echo "ERROR: ya hay otra operación de la demo en curso" >&2
        exit 2
    fi
    ;;
esac

launch() {   # nombre archivo_log script
    # Bajo supervisor: si el proceso se muere, vuelve solo a los 3 s. Un
    # tablero que se cae en silencio es peor que no tenerlo — te deja mirando
    # una pagina muerta creyendo que ves al robot.
    sudo docker exec -d jetson bash -c         "$ROS && while true; do python3 $D/$3 >> /tmp/$2 2>&1; sleep 3; done"
    echo "   $1"
}

launch_agent() {
    # El perfil llega como dato explícito al proceso supervisado. Así un
    # reinicio del agente no puede volver a habilitar ayudas por accidente.
    sudo docker exec -d \
        -e G1_OPERATION_PROFILE="$OPERATION_PROFILE" \
        jetson bash -c \
        "$ROS && while true; do python3 $D/agent/agent.py \
        >> /tmp/agent.log 2>&1; sleep 3; done"
    echo "   agente ($OPERATION_PROFILE)"
}

stop_safety() {
    sudo docker exec jetson pkill -f \
        "[n]av2_collision_monitor|[l]ifecycle_manager_safety" \
        2>/dev/null || true
}

start_safety() {
    stop_safety
    sudo docker exec -d jetson bash -c \
        "$ROS && while true; do \
        ros2 run nav2_collision_monitor collision_monitor \
        --ros-args --params-file $D/config/collision_monitor.yaml \
        >> /tmp/collision_monitor.log 2>&1; sleep 3; done"
    sleep 2
    sudo docker exec -d jetson bash -c \
        "$ROS && while true; do \
        ros2 run nav2_lifecycle_manager lifecycle_manager \
        --ros-args -r __node:=lifecycle_manager_safety \
        -p autostart:=true -p 'node_names:=[collision_monitor]' \
        -p use_sim_time:=true \
        >> /tmp/lifecycle_manager_safety.log 2>&1; sleep 3; done"
    echo "   seguridad de colisiones Nav2"
}

stop_navigation_stack() {
    # El bash desprendido ignora SIGINT. TERM detiene tanto al supervisor como
    # al lanzador; dejarlo vivo hizo que Nav2 sobreviviera a un reloj nuevo.
    sudo docker exec jetson pkill -TERM -f \
        "[n]avigation/nav2_stack.py" 2>/dev/null || true
    sleep 3
    # Launch puede morir antes que alguno de sus hijos. Limpiarlos evita dos
    # controladores publicando tras un reinicio, justo el fallo que originó la
    # autoridad exclusiva.
    sudo docker exec jetson pkill -TERM -f \
        "/nav2_controller/[c]ontroller_server|/nav2_planner/[p]lanner_server|/nav2_behaviors/[b]ehavior_server|/nav2_velocity_smoother/[v]elocity_smoother|/nav2_bt_navigator/[b]t_navigator|[l]ifecycle_manager_navigation" \
        2>/dev/null || true
    sleep 1
    if sudo docker exec jetson pgrep -f \
        "[n]avigation/nav2_stack.py|/nav2_controller/[c]ontroller_server|/nav2_planner/[p]lanner_server|/nav2_behaviors/[b]ehavior_server|/nav2_velocity_smoother/[v]elocity_smoother|/nav2_bt_navigator/[b]t_navigator|[l]ifecycle_manager_navigation" \
        >/dev/null 2>&1; then
        sudo docker exec jetson pkill -KILL -f \
            "[n]avigation/nav2_stack.py|/nav2_controller/[c]ontroller_server|/nav2_planner/[p]lanner_server|/nav2_behaviors/[b]ehavior_server|/nav2_velocity_smoother/[v]elocity_smoother|/nav2_bt_navigator/[b]t_navigator|[l]ifecycle_manager_navigation" \
            2>/dev/null || true
    fi
}

navigation_is_active() {
    sudo docker exec jetson pgrep -f \
        "/nav2_bt_navigator/[b]t_navigator" >/dev/null 2>&1 \
        || return 1
    sudo docker exec jetson bash -lc \
        "$ROS && timeout 5 ros2 lifecycle get /nav2/bt_navigator 2>/dev/null" \
        | grep -q "active"
}

start_navigation_stack() {
    if navigation_is_active; then
        echo "   Nav2 ya estaba activo"
        return 0
    fi
    stop_navigation_stack
    sudo docker exec jetson bash -c ": > /tmp/nav2_stack.log"
    sudo docker exec -d jetson bash -c \
        "$ROS && while true; do \
        python3 $D/navigation/nav2_stack.py \
            --params-file $D/config/nav2.yaml \
            >> /tmp/nav2_stack.log 2>&1; sleep 3; done"
    for _ in $(seq 1 30); do
        if navigation_is_active; then
            echo "   rutas y esquive Nav2"
            return 0
        fi
        sleep 1
    done
    echo "ERROR: Nav2 no llegó al estado activo" >&2
    sudo docker exec jetson tail -20 /tmp/nav2_stack.log >&2 || true
    return 1
}

stop_laser_scan() {
    sudo docker exec jetson pkill -f "laser_scan_adapter.py" \
        2>/dev/null || true
}

start_laser_scan() {
    if sudo docker exec jetson pgrep -f \
        "^python3 .*/laser_scan_adapter.py$" >/dev/null 2>&1; then
        return 0
    fi
    launch "adaptador LaserScan" laser_scan_adapter.log \
        navigation/laser_scan_adapter.py
}

stop_depth_pointcloud() {
    sudo docker exec jetson pkill -f "[p]oint_cloud_xyz_node" \
        2>/dev/null || true
}

start_depth_pointcloud() {
    if sudo docker exec jetson pgrep -f \
        "[p]oint_cloud_xyz_node" >/dev/null 2>&1; then
        return 0
    fi
    # ROS mantiene esta conversión estándar también para cámaras físicas: el
    # simulador sólo reemplaza al driver que publica profundidad y calibración.
    sudo docker exec -d jetson bash -c \
        "$ROS && while true; do \
        ros2 run depth_image_proc point_cloud_xyz_node --ros-args \
        -r image_rect:=/g1/head_cam/depth \
        -r camera_info:=/g1/head_cam/camera_info \
        -r points:=/g1/head_cam/points \
        -p use_sim_time:=true \
        >> /tmp/depth_pointcloud.log 2>&1; sleep 3; done"
    echo "   nube 3D de la cámara"
}

stop_mapping() {
    sudo docker exec jetson pkill -f \
        "online_async_launch.py|async_slam_toolbox_node" \
        2>/dev/null || true
}

start_mapping() {
    # Mapear y localizar publican la misma corrección map->odom. Son modos
    # excluyentes: dos fuentes simultáneas harían incoherente la posición.
    stop_navigation_stack
    stop_localization_stack
    stop_mapping
    start_laser_scan
    sudo docker exec -d jetson bash -c \
        "$ROS && while true; do \
        ros2 launch slam_toolbox online_async_launch.py \
        slam_params_file:=$D/config/slam_toolbox.yaml use_sim_time:=true \
        >> /tmp/slam_toolbox.log 2>&1; sleep 3; done"
    echo "   mapa SLAM Toolbox"
}

stop_localization_stack() {
    sudo docker exec jetson pkill -TERM -f \
        "[l]ocalization/localization_stack.py|[n]avigation/localization_stack.py" \
        2>/dev/null || true
    sleep 2
    sudo docker exec jetson pkill -TERM -f \
        "/nav2_map_server/[m]ap_server|/nav2_amcl/[a]mcl|[l]ifecycle_manager_localization" \
        2>/dev/null || true
}

localization_is_active() {
    sudo docker exec jetson pgrep -f \
        "/nav2_amcl/[a]mcl" >/dev/null 2>&1 \
        || return 1
    sudo docker exec jetson bash -lc \
        "$ROS && timeout 5 ros2 lifecycle get /nav2/amcl 2>/dev/null" \
        | grep -q "active"
}

start_localization_stack() {
    if localization_is_active; then
        echo "   mapa fijo y localización ya estaban activos"
        return 0
    fi
    stop_navigation_stack
    stop_mapping
    stop_localization_stack
    start_laser_scan
    sudo docker exec jetson bash -c ": > /tmp/localization_stack.log"
    sudo docker exec -d jetson bash -c \
        "$ROS && while true; do \
        python3 $D/navigation/localization_stack.py \
            --params-file $D/config/localization.yaml \
            >> /tmp/localization_stack.log 2>&1; sleep 3; done"
    for _ in $(seq 1 30); do
        if localization_is_active; then
            echo "   mapa fijo y localización AMCL"
            return 0
        fi
        sleep 1
    done
    echo "ERROR: la localización no llegó al estado activo" >&2
    sudo docker exec jetson tail -30 /tmp/localization_stack.log >&2 || true
    return 1
}

stop_layers() {
    sudo docker exec jetson pkill -f \
        "mobility_authority.py|stand_hold.py|nav2_adapter.py|go_to.py|align_with_table.py|detector.py|object_detector.py|open_vocabulary_detector.py|table_localizer.py|detection_adapter.py|agent.py" \
        2>/dev/null
}

start_layers() {
    # La barrera final nace antes que la autoridad. Si cualquier capa tarda o
    # se cae, el watchdog del robot recibe cero en vez del último comando.
    start_safety
    start_depth_pointcloud
    launch "autoridad"            mobility.log          mobility_authority.py
    launch "quieto"               stand.log             stand_hold.py
    launch "adaptador Nav2"       nav2_adapter.log      skills/nav2_adapter.py
    launch "alineacion fina"      alignment.log         skills/align_with_table.py
    launch "detector RT-DETR"     object_detector.log   skills/object_detector.py
    launch "búsqueda visual"      open_vocabulary.log   skills/open_vocabulary_detector.py
    launch "posición 3D"          table_localizer.log   skills/table_localizer.py
    launch "adaptador percepción" detection_adapter.log skills/detection_adapter.py
    launch_agent
}

preflight() {
    # Verificar y limpiar ANTES de arrancar, no despues de que falle.
    echo ">> Revisando que no haya restos de corridas anteriores..."
    local zombis
    zombis=$(pgrep -f "g1_robot.p[y]" 2>/dev/null | wc -l)
    if [ "$zombis" -gt 0 ]; then
        echo "   había $zombis proceso(s) de la cadena de Isaac: limpiando"
        bash ~/go2-lab/g1/run_g1.sh stop >/dev/null || {
            echo "ERROR: no se pudo detener la instancia anterior" >&2
            exit 1
        }
    fi
    stop_navigation_stack
    stop_localization_stack
    stop_mapping
    stop_laser_scan
    stop_depth_pointcloud
    stop_layers
    stop_safety
    sleep 2

    local gpu
    gpu=$(nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits)
    if [ "$gpu" -gt 500 ]; then
        echo "   AVISO: la GPU tiene $gpu MiB ocupados por otro proceso"
        nvidia-smi --query-compute-apps=pid,used_memory --format=csv,noheader
    fi
    echo "   limpio (robot: 0 instancias, GPU: ${gpu} MiB)"

    # El tablero va aparte y sobrevive a todo; si no esta, lo prendemos.
    if ! sudo docker exec jetson pgrep -f dashboard.py >/dev/null 2>&1; then
        echo ">> El tablero no estaba prendido: lo prendo"
        launch "tablero" dashboard.log dashboard/dashboard.py
        sleep 3
    fi
}

case "${1:-up}" in
up)
    printf '%s\n' "$OPERATION_PROFILE" > "$OPERATION_PROFILE_FILE"
    preflight
    echo ">> El robot (Isaac, ~1 min de arranque)..."
    ARM_CONTROLLER="${G1_ARM_CONTROLLER:-pose}"
    case "$ARM_CONTROLLER" in
        pose|pink) ;;
        *) echo "ERROR: G1_ARM_CONTROLLER debe ser pose o pink" >&2; exit 2 ;;
    esac
    SCENE_OPTIONS="--camera --scene --lidar --visible --arm-controller $ARM_CONTROLLER"
    # El mapa base se construye sin objetos temporales. La escena normal los
    # incluye para demostrar que el LiDAR los descubre después de mapear.
    if [ "${G1_NAVIGATION_OBSTACLE:-1}" = "1" ]; then
        SCENE_OPTIONS="$SCENE_OPTIONS --navigation-obstacle"
        if [ -n "${G1_NAVIGATION_OBSTACLE_HEIGHT:-}" ]; then
            if ! [[ "$G1_NAVIGATION_OBSTACLE_HEIGHT" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
                echo "ERROR: G1_NAVIGATION_OBSTACLE_HEIGHT debe ser un número positivo" >&2
                exit 2
            fi
            SCENE_OPTIONS="$SCENE_OPTIONS --navigation-obstacle-height ${G1_NAVIGATION_OBSTACLE_HEIGHT}"
        fi
    fi
    if ! ROBOT_LAUNCH_OUTPUT=$(
        bash ~/go2-lab/g1/run_g1.sh wbc 29dof 0 \
            "$SCENE_OPTIONS" 2>&1
    ); then
        echo "$ROBOT_LAUNCH_OUTPUT" >&2
        echo "ERROR: Isaac no fue lanzado; no se acepta un log anterior" >&2
        exit 1
    fi
    echo "$ROBOT_LAUNCH_OUTPUT" | tail -1

    echo ">> Esperando a que el robot este en pie..."
    until tr '\r' '\n' < ~/g1.log 2>/dev/null | grep -q "LISTO:"; do
        pgrep -f "g1_robot.p[y]" >/dev/null || { echo "   el robot no arranco"; exit 1; }
        sleep 5
    done
    echo "   listo"

    echo ">> Las capas de arriba, en la jetson:"
    stop_layers
    sleep 2
    start_layers
    echo ">> Mapa fijo y localización en la Jetson:"
    start_localization_stack
    sleep 4

    echo ""
    echo "TODO CARGADO Y EL ROBOT CONGELADO, quieto en el punto de partida."
    echo "Acomoda el cliente de Isaac y el tablero, y cuando quieras:"
    echo "En Isaac: Perspective; rueda=zoom, boton central=mover, Alt+izq=girar,"
    echo "           seleccionar robot + F=centrarlo. No mover /head_cam."
    echo ""
    echo "  bash run_demo.sh start      suelta el robot (la policy toma el control)"
    echo "  bash run_demo.sh freeze     lo vuelve a congelar en el origen (instantaneo)"
    echo ""
    echo "Despues, la escalera de pruebas EN ESTE ORDEN:"
    echo "  1. bash run_demo.sh check stand    se queda de pie? (60 s)"
    echo "  2. bash run_demo.sh check walk     camina y frena? (1 min)"
    echo "  3. bash run_demo.sh check goto     llega a un punto? (3 min)"
    echo "  4. bash run_demo.sh mission        recien ahi, la mision completa"
    ;;

layers)
    # Permite reconstruir la computadora de a bordo sin pagar otro arranque de
    # Isaac. El robot debe seguir corriendo en el host.
    pgrep -f "g1_robot.p[y]" >/dev/null \
        || { echo "el robot no está corriendo; usar: bash run_demo.sh up"; exit 1; }
    stop_navigation_stack
    stop_layers
    sleep 2
    if ! sudo docker exec jetson pgrep -f dashboard.py >/dev/null 2>&1; then
        launch "tablero" dashboard.log dashboard/dashboard.py
    fi
    start_layers
    sleep 6
    if sudo docker exec jetson bash -lc \
        "$ROS && timeout 5 ros2 topic echo --once /g1/odom >/dev/null"; then
        start_localization_stack
        start_navigation_stack
    else
        echo "Nav2 espera a que el robot sea soltado con: bash run_demo.sh start"
    fi
    echo "capas de la Jetson reiniciadas sin recargar Isaac"
    ;;

map)
    case "${2:-status}" in
    on)
        pgrep -f "g1_robot.p[y]" >/dev/null \
            || { echo "el robot no está corriendo"; exit 1; }
        start_mapping
        sleep 5
        echo "mapeo prendido; verificar: bash run_demo.sh map check"
        ;;
    off)
        stop_navigation_stack
        stop_mapping
        echo "mapeo apagado; el LaserScan queda disponible para seguridad"
        ;;
    check)
        sudo docker exec jetson bash -lc \
            "$ROS && python3 $D/tools/check_time_and_tf.py \
            && python3 $D/tools/check_laser_scan.py \
            && python3 $D/tools/check_slam_map.py"
        ;;
    status)
        for process in laser_scan_adapter async_slam_toolbox_node; do
            if sudo docker exec jetson pgrep -f "$process" >/dev/null 2>&1; then
                echo "$process: corriendo"
            else
                echo "$process: DETENIDO"
            fi
        done
        navigation_is_active \
            && echo "Nav2: activo" \
            || echo "Nav2: espera al robot"
        ;;
    *) echo "uso: bash run_demo.sh map {on|off|check|status}"; exit 1 ;;
    esac
    ;;

localize)
    case "${2:-status}" in
    on)
        pgrep -f "g1_robot.p[y]" >/dev/null \
            || { echo "el robot no está corriendo"; exit 1; }
        start_localization_stack
        ;;
    off)
        stop_navigation_stack
        stop_localization_stack
        echo "localización apagada"
        ;;
    check)
        sudo docker exec jetson bash -lc \
            "$ROS && python3 $D/tools/check_time_and_tf.py \
            && python3 $D/tools/check_laser_scan.py \
            && python3 $D/tools/check_slam_map.py \
            && timeout 10 ros2 topic echo --once /amcl_pose >/dev/null"
        if navigation_is_active; then
            sudo docker exec jetson bash -lc \
                "$ROS && python3 $D/tools/check_local_costmap.py"
        fi
        ;;
    status)
        localization_is_active \
            && echo "AMCL: activo sobre el mapa fijo" \
            || echo "AMCL: DETENIDO"
        ;;
    *) echo "uso: bash run_demo.sh localize {on|off|check|status}"; exit 1 ;;
    esac
    ;;

start)
    # Suelta el robot: la policy toma el control con la memoria limpia.
    sudo docker exec jetson bash -c         "$ROS && timeout 8 ros2 topic pub --once /g1/control std_msgs/msg/String \"{data: start}\""         >/dev/null 2>&1
    echo "robot SOLTADO; espero reloj y posición antes de entregar rutas..."
    start_localization_stack
    if ! sudo docker exec jetson bash -lc \
        "$ROS && python3 $D/tools/check_time_and_tf.py"; then
        echo "ERROR: el robot no publicó una posición utilizable" >&2
        exit 1
    fi
    if ! sudo docker exec jetson bash -lc \
        "$ROS && timeout 20 ros2 topic echo --once /map >/dev/null"; then
        echo "ERROR: el mapa no apareció después de soltar el robot" >&2
        exit 1
    fi
    start_navigation_stack
    echo "robot y Nav2 ACTIVOS. Congelar: bash run_demo.sh freeze"
    ;;

freeze)
    # Lo congela de nuevo en el punto de partida, sin recargar Isaac.
    sudo docker exec jetson bash -c         "$ROS && timeout 8 ros2 topic pub --once /g1/control std_msgs/msg/String \"{data: freeze}\""         >/dev/null 2>&1
    echo "robot CONGELADO en el origen. Soltarlo: bash run_demo.sh start"
    ;;

pose)
    POSE="${2:-}"
    case "$POSE" in
    reposo|listo|transporte) ;;
    *)
        echo "uso: bash run_demo.sh pose {reposo|listo|transporte}"
        exit 1
        ;;
    esac
    # Publicar no prueba que el robot haya recibido ni ejecutado la orden.
    # Este verificador espera confirmación y compara el objetivo con los
    # ángulos reales de las catorce articulaciones.
    sudo docker exec jetson bash -c \
        "$ROS && python3 $D/tools/set_arm_pose.py $POSE"
    ;;

payload)
    ACTION="${2:-}"
    case "$ACTION" in
    attach)
        [ -n "${3:-}" ] || {
            echo "uso: bash run_demo.sh payload attach <kg>"
            exit 1
        }
        sudo docker exec jetson bash -c \
            "$ROS && python3 $D/tools/set_payload.py attach '$3'"
        ;;
    detach)
        sudo docker exec jetson bash -c \
            "$ROS && python3 $D/tools/set_payload.py detach"
        ;;
    *)
        echo "uso: bash run_demo.sh payload {attach <kg>|detach}"
        exit 1
        ;;
    esac
    ;;

tablero)
    # El tablero vive aparte del robot: se prende una vez y se queda.
    case "${2:-on}" in
    on)
        sudo docker exec jetson pgrep -f dashboard.py >/dev/null 2>&1             && { echo "el tablero ya estaba prendido"; exit 0; }
        launch "tablero" dashboard.log dashboard/dashboard.py
        sleep 3
        echo "tablero prendido. Desde tu maquina:"
        echo "  ssh -L 8080:localhost:8080 lucas@<IP>   y abrir http://localhost:8080"
        ;;
    off)
        sudo docker exec jetson pkill -f dashboard.py 2>/dev/null
        # Esperar evita que un "off && on" confunda al proceso viejo, todavía
        # visible durante unos milisegundos, con el tablero recién iniciado.
        for _ in 1 2 3 4 5; do
            sudo docker exec jetson pgrep -f dashboard.py >/dev/null 2>&1 \
                || break
            sleep 1
        done
        echo "tablero apagado"
        ;;
    esac
    ;;

check)
    # La escalera de pruebas, de lo mas simple a lo mas complejo.
    if [ "${2:-}" = "safety" ]; then
        sudo docker exec jetson bash -c \
            "$ROS && python3 $D/tools/check_collision_safety.py wiring"
        exit $?
    fi
    if [ "${2:-}" = "safety-wall" ]; then
        sudo docker exec jetson bash -c \
            "$ROS && timeout 8 ros2 topic pub --once /g1/control \
            std_msgs/msg/String \"{data: start}\"" >/dev/null 2>&1
        sleep 2
        sudo docker exec jetson bash -c \
            "$ROS && python3 $D/tools/check_collision_safety.py wall"
        exit $?
    fi
    if [ "${2:-}" = "clock" ]; then
        sudo docker exec jetson bash -c \
            "$ROS && python3 $D/tools/check_clock.py"
        exit $?
    fi
    if [ "${2:-}" = "home" ]; then
        sudo docker exec jetson bash -c \
            "$ROS && python3 $D/tools/check_home_return.py"
        exit $?
    fi
    if [ "${2:-}" = "obstacle" ]; then
        navigation_is_active || {
            echo "Nav2 no está activo; primero: bash run_demo.sh start" >&2
            exit 1
        }
        sudo docker exec jetson bash -c \
            "$ROS && python3 $D/tools/check_obstacle_navigation.py"
        exit $?
    fi
    # Soltar antes evita que un robot clavado artificialmente apruebe quietud.
    sudo docker exec jetson bash -c         "$ROS && timeout 8 ros2 topic pub --once /g1/control std_msgs/msg/String \"{data: start}\""         >/dev/null 2>&1
    # La confirmación se publica desde el lazo de física; el proceso de prueba
    # no debe ganarle la carrera y leer todavía el estado congelado anterior.
    sleep 2
    if [ -n "${3:-}" ]; then
        sudo docker exec jetson bash -c \
            "$ROS && python3 $D/tools/checks.py ${2:-all} '$3'"
    else
        sudo docker exec jetson bash -c \
            "$ROS && python3 $D/tools/checks.py ${2:-all}"
    fi
    ;;

clock)
    # El verificador espera la confirmación real de freeze/start antes de
    # mandar el objetivo. Un plazo fijo produjo una carrera: navegación llegó
    # a tomar la autoridad mientras el robot todavía cambiaba de estado.
    sudo docker exec jetson bash -c \
        "$ROS && python3 $D/tools/check_clock.py"
    ;;

read-clock)
    EXPECTED="${2:-09:00}"
    sudo docker exec jetson bash -c \
        "$ROS && python3 $D/tools/check_clock_reading.py \
        --expected '$EXPECTED' --repetitions 3"
    ;;

table)
    COLOR="${2:-red}"
    sudo docker exec jetson bash -c \
        "$ROS && python3 $D/tools/check_table_detection.py '$COLOR'"
    ;;

align-table)
    COLOR="${2:-red}"
    sudo docker exec jetson bash -c \
        "$ROS && python3 $D/tools/check_table_alignment.py '$COLOR'"
    ;;

search-table)
    COLOR="${2:-red}"
    sudo docker exec jetson bash -c \
        "$ROS && python3 $D/tools/check_table_detection.py \
        '$COLOR' --current-view"
    ;;

mission)
    MISSION="${2:-andá al reloj, leé la hora, elegí la mesa correcta, simulá el agarre agregando la carga física aprobada de 0,5 kg y volvé al inicio}"
    sudo docker exec jetson bash -c \
        "$ROS && timeout 8 ros2 topic pub --once /g1/mission std_msgs/msg/String \
        \"{data: '$MISSION'}\"" \
        >/dev/null 2>&1
    echo "mision enviada: $MISSION"
    echo "seguila en el tablero, o con: bash run_demo.sh status"
    ;;

reset)
    # Devuelve el robot al punto de partida y reinicia las capas de arriba,
    # para poder repetir la mision desde cero sin relanzar el simulador.
    sudo docker exec jetson bash -c         "$ROS && timeout 8 ros2 topic pub --once /g1/reset std_msgs/msg/String \"{data: ya}\""         >/dev/null 2>&1
    stop_navigation_stack
    stop_localization_stack
    sudo docker exec jetson pkill -f \
        "nav2_adapter.py|go_to.py|align_with_table.py|detector.py|object_detector.py|open_vocabulary_detector.py|table_localizer.py|detection_adapter.py|agent.py" \
        2>/dev/null
    sleep 2
    launch "adaptador Nav2"       nav2_adapter.log      skills/nav2_adapter.py
    launch "alineacion fina"      alignment.log         skills/align_with_table.py
    launch "detector RT-DETR"     object_detector.log   skills/object_detector.py
    launch "búsqueda visual"      open_vocabulary.log   skills/open_vocabulary_detector.py
    launch "posición 3D"          table_localizer.log   skills/table_localizer.py
    launch "adaptador percepción" detection_adapter.log skills/detection_adapter.py
    launch_agent
    # Un teletransporte invalida la posición anterior, pero no el mapa fijo.
    # Reiniciar AMCL fuerza una nueva localización desde el origen conocido.
    start_localization_stack
    if sudo docker exec jetson bash -lc \
        "$ROS && python3 $D/tools/check_time_and_tf.py"; then
        start_navigation_stack
    else
        echo "ERROR: el reinicio no recuperó posición y coordenadas" >&2
        exit 1
    fi
    echo "todo reiniciado. Darle la mision: bash run_demo.sh mission"
    ;;

status)
    echo "== la versión =="
    if repo_root=$(
        git -C "$HOST_DEMO_DIR" rev-parse --show-toplevel 2>/dev/null
    ); then
        branch=$(git -C "$repo_root" branch --show-current)
        commit=$(git -C "$repo_root" rev-parse --short=12 HEAD)
        if [ -n "$(git -C "$repo_root" status --porcelain)" ]; then
            cleanliness="CON CAMBIOS LOCALES"
        else
            cleanliness="limpia"
        fi
        echo "  $branch @ $commit · $cleanliness"
    else
        echo "  SIN GIT: no se puede identificar el código ejecutado"
    fi
    echo "== el robot =="
    echo "  perfil: $OPERATION_PROFILE"
    pgrep -f "g1_robot.p[y]" >/dev/null && echo "  corriendo" || echo "  detenido"
    tr '\r' '\n' < ~/g1.log 2>/dev/null | grep RTF | tail -1 | sed 's/^/  /'
    echo "== las capas de arriba =="
    for p in mobility_authority stand_hold nav2_adapter align_with_table object_detector open_vocabulary_detector table_localizer detection_adapter agent dashboard; do
        if sudo docker exec jetson \
            pgrep -f "^python3 .*/$p.py$" >/dev/null 2>&1; then
            echo "  $p: corriendo"
        else
            echo "  $p: DETENIDO"
        fi
    done
    echo "== la seguridad =="
    for p in nav2_collision_monitor lifecycle_manager_safety; do
        if sudo docker exec jetson pgrep -f "$p" >/dev/null 2>&1; then
            echo "  $p: corriendo"
        else
            echo "  $p: DETENIDO"
        fi
    done
    echo "== mapa y posición =="
    for p in laser_scan_adapter point_cloud_xyz_node async_slam_toolbox_node amcl map_server; do
        if sudo docker exec jetson pgrep -f "$p" >/dev/null 2>&1; then
            echo "  $p: corriendo"
        else
            echo "  $p: DETENIDO"
        fi
    done
    if navigation_is_active; then
        echo "  Nav2: activo (ruta global + esquive local)"
    else
        echo "  Nav2: DETENIDO o esperando que se suelte el robot"
    fi
    perception_status=$(
        sudo docker exec jetson bash -lc \
            "$ROS && timeout 5 ros2 topic echo --once --field data /g1/perception/status" \
            2>/dev/null | head -1
    )
    if [ -n "$perception_status" ]; then
        echo "  detector vivo: $perception_status"
    else
        echo "  detector SIN DATOS: el proceso existe pero no respondió en 5 s"
    fi
    echo "== la mision =="
    mission_state=$(
        sudo docker exec jetson bash -lc \
            "$ROS && timeout 5 ros2 topic echo --once --field data /g1/mission_state" \
            2>/dev/null | head -1
    )
    mission_phase=$(
        printf '%s' "$mission_state" | python3 -c \
            'import json, sys; print(json.load(sys.stdin).get("state", ""))' \
            2>/dev/null || true
    )
    active_step=$(
        printf '%s' "$mission_state" | python3 -c \
            'import json, sys; print(json.load(sys.stdin).get("active_step_id") or "")' \
            2>/dev/null || true
    )
    if [ "$mission_phase" = "idle" ]; then
        echo "  sin misión activa; el agente está esperando"
    elif [ -n "$mission_phase" ]; then
        echo "  estado vivo: $mission_phase"
        if [ -n "$active_step" ]; then
            echo "  paso activo: $active_step"
        fi
    else
        # Un archivo de log puede contener una misión vieja. Si el mensaje
        # vivo falta, es más honesto declararlo que mostrar historia obsoleta.
        echo "  SIN DATOS VIVOS del agente"
    fi
    ;;

kill)
    # MATAR TODO: robot + capas de arriba. El tablero sigue prendido para poder
    # ver como el sistema vuelve a levantarse (se apaga con: tablero off).
    bash ~/go2-lab/g1/run_g1.sh stop
    stop_navigation_stack
    stop_layers
    stop_safety
    stop_mapping
    stop_localization_stack
    stop_laser_scan
    stop_depth_pointcloud
    sleep 2
    echo "robot: $(pgrep -f 'g1_robot.p[y]' | wc -l) instancias vivas"
    echo "GPU:   $(nvidia-smi --query-gpu=memory.used --format=csv,noheader)"
    echo "el tablero sigue prendido (apagarlo: bash run_demo.sh tablero off)"
    ;;

down)
    # La autoridad y stand_hold pertenecen al robot a bordo: siguen activos
    # mientras el robot esté de pie. Sólo se detienen tareas y percepción.
    stop_navigation_stack
    sudo docker exec jetson pkill -f \
        "nav2_adapter.py|go_to.py|align_with_table.py|detector.py|object_detector.py|open_vocabulary_detector.py|table_localizer.py|detection_adapter.py|agent.py"
    echo "misión detenida (robot, autoridad, stand_hold y tablero siguen)"
    ;;

*) echo "uso: $0 {up|layers|map [on|off|check|status]|localize [on|off|check|status]|start|freeze|clock|read-clock [HH:MM]|table [red|blue]|search-table [red|blue]|pose [reposo|listo|transporte]|payload [attach KG|detach]|check [safety|safety-wall|authority|stand|walk|turn|goto|clock|home|obstacle|all]|mission [texto]|kill|tablero [on|off]|status|down}"; exit 1;;
esac
