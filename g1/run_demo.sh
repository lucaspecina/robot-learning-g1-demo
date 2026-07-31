#!/usr/bin/env bash
# Levanta el sistema completo de la demo, en orden, y deja todo listo para mirar.
#
# Uso (en la VM):
#   bash run_demo.sh up          arranca robot + skills + agente
#   bash run_demo.sh check [cual] LA ESCALERA: stand | walk | goto | clock | home
#   bash run_demo.sh clock      va al reloj conocido y termina mirándolo
#   bash run_demo.sh read-clock lee el recorte vivo mediante el servidor
#   bash run_demo.sh table red  mira una mesa desde una pose sólo de prueba
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

launch() {   # nombre archivo_log script
    # Bajo supervisor: si el proceso se muere, vuelve solo a los 3 s. Un
    # tablero que se cae en silencio es peor que no tenerlo — te deja mirando
    # una pagina muerta creyendo que ves al robot.
    sudo docker exec -d jetson bash -c         "$ROS && while true; do python3 $D/$3 >> /tmp/$2 2>&1; sleep 3; done"
    echo "   $1"
}

stop_layers() {
    sudo docker exec jetson pkill -f \
        "mobility_authority.py|stand_hold.py|go_to.py|detector.py|object_detector.py|open_vocabulary_detector.py|table_localizer.py|detection_adapter.py|agent.py" \
        2>/dev/null
}

start_layers() {
    # El árbitro nace antes que cualquier fuente. Así nunca existe una ventana
    # donde navegación o pruebas puedan hablar directo con el robot.
    launch "autoridad"            mobility.log          mobility_authority.py
    launch "quieto"               stand.log             stand_hold.py
    launch "navegacion"           goto.log              skills/go_to.py
    launch "detector RT-DETR"     object_detector.log   skills/object_detector.py
    launch "búsqueda visual"      open_vocabulary.log   skills/open_vocabulary_detector.py
    launch "posición 3D"          table_localizer.log   skills/table_localizer.py
    launch "adaptador percepción" detection_adapter.log skills/detection_adapter.py
    launch "agente"               agent.log             agent/agent.py
}

preflight() {
    # Verificar y limpiar ANTES de arrancar, no despues de que falle.
    echo ">> Revisando que no haya restos de corridas anteriores..."
    local zombis
    zombis=$(pgrep -f "g1_robot.p[y]" 2>/dev/null | wc -l)
    if [ "$zombis" -gt 0 ]; then
        echo "   había $zombis proceso(s) de la cadena de Isaac: limpiando"
        bash ~/go2-lab/g1/run_g1.sh stop >/dev/null
    fi
    stop_layers
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
    preflight
    echo ">> El robot (Isaac, ~1 min de arranque)..."
    bash ~/go2-lab/g1/run_g1.sh wbc 29dof 0 "--camera --scene --visible" | tail -1

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
    sleep 6

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
    stop_layers
    sleep 2
    if ! sudo docker exec jetson pgrep -f dashboard.py >/dev/null 2>&1; then
        launch "tablero" dashboard.log dashboard/dashboard.py
    fi
    start_layers
    sleep 6
    echo "capas de la Jetson reiniciadas sin recargar Isaac"
    ;;

start)
    # Suelta el robot: la policy toma el control con la memoria limpia.
    sudo docker exec jetson bash -c         "$ROS && timeout 8 ros2 topic pub --once /g1/control std_msgs/msg/String \"{data: start}\""         >/dev/null 2>&1
    echo "robot SOLTADO. Volver a congelarlo: bash run_demo.sh freeze"
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

search-table)
    COLOR="${2:-red}"
    sudo docker exec jetson bash -c \
        "$ROS && python3 $D/tools/check_table_detection.py \
        '$COLOR' --current-view"
    ;;

mission)
    MISSION="${2:-andá al reloj, leé la hora, elegí la mesa correcta, agarrá el objeto y volvé al inicio}"
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
    sudo docker exec jetson pkill -f \
        "go_to.py|detector.py|object_detector.py|open_vocabulary_detector.py|table_localizer.py|detection_adapter.py|agent.py" \
        2>/dev/null
    sleep 2
    launch "navegacion"           goto.log              skills/go_to.py
    launch "detector RT-DETR"     object_detector.log   skills/object_detector.py
    launch "búsqueda visual"      open_vocabulary.log   skills/open_vocabulary_detector.py
    launch "posición 3D"          table_localizer.log   skills/table_localizer.py
    launch "adaptador percepción" detection_adapter.log skills/detection_adapter.py
    launch "agente"               agent.log             agent/agent.py
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
    pgrep -f "g1_robot.p[y]" >/dev/null && echo "  corriendo" || echo "  detenido"
    tr '\r' '\n' < ~/g1.log 2>/dev/null | grep RTF | tail -1 | sed 's/^/  /'
    echo "== las capas de arriba =="
    for p in mobility_authority stand_hold go_to object_detector open_vocabulary_detector table_localizer detection_adapter agent dashboard; do
        if sudo docker exec jetson \
            pgrep -f "^python3 .*/$p.py$" >/dev/null 2>&1; then
            echo "  $p: corriendo"
        else
            echo "  $p: DETENIDO"
        fi
    done
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
    stop_layers
    sleep 2
    echo "robot: $(pgrep -f 'g1_robot.p[y]' | wc -l) instancias vivas"
    echo "GPU:   $(nvidia-smi --query-gpu=memory.used --format=csv,noheader)"
    echo "el tablero sigue prendido (apagarlo: bash run_demo.sh tablero off)"
    ;;

down)
    # La autoridad y stand_hold pertenecen al robot a bordo: siguen activos
    # mientras el robot esté de pie. Sólo se detienen tareas y percepción.
    sudo docker exec jetson pkill -f \
        "go_to.py|detector.py|object_detector.py|open_vocabulary_detector.py|table_localizer.py|detection_adapter.py|agent.py"
    echo "misión detenida (robot, autoridad, stand_hold y tablero siguen)"
    ;;

*) echo "uso: $0 {up|layers|start|freeze|clock|read-clock [HH:MM]|table [red|blue]|search-table [red|blue]|pose [reposo|listo|transporte]|payload [attach KG|detach]|check [authority|stand|walk|turn|goto|clock|home|all]|mission [texto]|kill|tablero [on|off]|status|down}"; exit 1;;
esac
