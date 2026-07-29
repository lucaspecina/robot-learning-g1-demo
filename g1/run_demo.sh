#!/usr/bin/env bash
# Levanta el sistema completo de la demo, en orden, y deja todo listo para mirar.
#
# Uso (en la VM):
#   bash run_demo.sh up          arranca robot + skills + agente
#   bash run_demo.sh check [cual] LA ESCALERA DE PRUEBAS: stand | walk | goto | all
#   bash run_demo.sh clock      va al reloj conocido y termina mirándolo
#   bash run_demo.sh pose NOMBRE  mueve brazos: reposo | listo | transporte
#   bash run_demo.sh mission     le da la mision completa de 10 pasos
#   bash run_demo.sh status      como viene todo
#   bash run_demo.sh down        apaga las capas de arriba (no el robot)
#
# El orden correcto es: up -> check all -> recien ahi mission. La mision de 10
# pasos no significa nada si el robot no se sostiene de pie.
#
# EL TABLERO VA APARTE, a proposito: es un observador, no parte del robot.
# Se prende una vez (tablero on) y se queda vivo mientras el robot nace, se
# cae y vuelve a arrancar — que es justo cuando uno mas necesita mirarlo.
#
# Para mirarlo desde tu maquina, dos ventanas:
#   - el tablero:  ssh -L 8080:localhost:8080 lucas@<IP>  -> http://localhost:8080
#   - el robot:    cliente de Isaac apuntando a la IP de la VM
set -uo pipefail

D=/workspace/g1          # ruta dentro del contenedor jetson
ROS="source /opt/ros/jazzy/setup.bash"

launch() {   # nombre archivo_log script
    # Bajo supervisor: si el proceso se muere, vuelve solo a los 3 s. Un
    # tablero que se cae en silencio es peor que no tenerlo — te deja mirando
    # una pagina muerta creyendo que ves al robot.
    sudo docker exec -d jetson bash -c         "$ROS && while true; do python3 $D/$3 >> /tmp/$2 2>&1; sleep 3; done"
    echo "   $1"
}

preflight() {
    # Verificar y limpiar ANTES de arrancar, no despues de que falle.
    echo ">> Revisando que no haya restos de corridas anteriores..."
    local zombis
    zombis=$(pgrep -f "g1_robot.p[y]" 2>/dev/null | wc -l)
    if [ "$zombis" -gt 0 ]; then
        echo "   habia $zombis instancia(s) del robot dando vueltas: matando"
        bash ~/go2-lab/g1/run_g1.sh stop >/dev/null
    fi
    sudo docker exec jetson pkill -f \
        "mobility_authority.py|stand_hold.py|go_to.py|detector.py|agent.py" \
        2>/dev/null
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
    sudo docker exec jetson pkill -f \
        "mobility_authority.py|stand_hold.py|go_to.py|detector.py|agent.py" \
        2>/dev/null
    sleep 2
    # El árbitro nace antes que cualquier fuente. Así nunca existe una ventana
    # donde navegación o pruebas puedan hablar directo con el robot.
    launch "autoridad"    mobility.log  mobility_authority.py
    launch "quieto"       stand.log     stand_hold.py
    launch "navegacion"  goto.log      skills/go_to.py
    launch "percepcion"  detector.log  skills/detector.py
    launch "agente"      agent.log     agent/agent.py
    sleep 6

    echo ""
    echo "TODO CARGADO Y EL ROBOT CONGELADO, quieto en el punto de partida."
    echo "Acomoda el cliente de Isaac y el tablero, y cuando quieras:"
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
    # Soltar antes evita que un robot clavado artificialmente apruebe quietud.
    sudo docker exec jetson bash -c         "$ROS && timeout 8 ros2 topic pub --once /g1/control std_msgs/msg/String \"{data: start}\""         >/dev/null 2>&1
    # La confirmación se publica desde el lazo de física; el proceso de prueba
    # no debe ganarle la carrera y leer todavía el estado congelado anterior.
    sleep 2
    sudo docker exec jetson bash -c         "$ROS && python3 $D/tools/checks.py ${2:-all}"
    ;;

clock)
    # Esta prueba aísla navegación, orientación y cámara. No usa todavía el
    # agente ni la lectura de hora, porque mezclar esas capas ocultaría cuál
    # falló si el reloj no termina dentro de la imagen.
    sudo docker exec jetson bash -c \
        "$ROS && timeout 8 ros2 topic pub --once /g1/control std_msgs/msg/String \"{data: freeze}\"" \
        >/dev/null 2>&1
    sleep 1
    sudo docker exec jetson bash -c \
        "$ROS && timeout 8 ros2 topic pub --once /g1/control std_msgs/msg/String \"{data: start}\"" \
        >/dev/null 2>&1
    sleep 2
    sudo docker exec jetson bash -c \
        "$ROS && timeout 8 ros2 topic pub --once /g1/goal geometry_msgs/msg/PoseStamped \
        \"{header: {frame_id: odom}, pose: {position: {x: 0.8, y: 1.8, z: 0.0}, \
        orientation: {x: 0.0, y: 0.0, z: 0.936109, w: 0.351709}}}\"" \
        >/dev/null 2>&1
    echo "prueba del reloj iniciada: llegada, orientación final y cámara"
    echo "seguí el movimiento en Isaac y la imagen en el tablero"
    ;;

mission)
    MISSION="${2:-fijate la hora en el reloj y llevale la botella a quien corresponda}"
    sudo docker exec jetson bash -c \
        "$ROS && timeout 8 ros2 topic pub --once /g1/mission std_msgs/msg/String \"{data: $MISSION}\"" \
        >/dev/null 2>&1
    echo "mision enviada: $MISSION"
    echo "seguila en el tablero, o con: bash run_demo.sh status"
    ;;

reset)
    # Devuelve el robot al punto de partida y reinicia las capas de arriba,
    # para poder repetir la mision desde cero sin relanzar el simulador.
    sudo docker exec jetson bash -c         "$ROS && timeout 8 ros2 topic pub --once /g1/reset std_msgs/msg/String \"{data: ya}\""         >/dev/null 2>&1
    sudo docker exec jetson pkill -f "go_to.py|detector.py|agent.py" 2>/dev/null
    sleep 2
    launch "navegacion"  goto.log      skills/go_to.py
    launch "percepcion"  detector.log  skills/detector.py
    launch "agente"      agent.log     agent/agent.py
    echo "todo reiniciado. Darle la mision: bash run_demo.sh mission"
    ;;

status)
    echo "== el robot =="
    pgrep -f "g1_robot.p[y]" >/dev/null && echo "  corriendo" || echo "  detenido"
    tr '\r' '\n' < ~/g1.log 2>/dev/null | grep RTF | tail -1 | sed 's/^/  /'
    echo "== las capas de arriba =="
    for p in mobility_authority stand_hold go_to detector agent dashboard; do
        if sudo docker exec jetson pgrep -f "$p.py" >/dev/null 2>&1; then
            echo "  $p: corriendo"
        else
            echo "  $p: DETENIDO"
        fi
    done
    echo "== la mision =="
    # El cierre normal de ROS deja una traza técnica en el archivo. Mostrar
    # sólo los mensajes del agente evita presentar ese residuo como una falla.
    sudo docker exec jetson sh -c \
        "grep -F '[agent]:' /tmp/agent.log 2>/dev/null | tail -6" \
        | sed 's/\[INFO\] \[[0-9.]*\] \[agent\]: /  /'
    ;;

kill)
    # MATAR TODO: robot + capas de arriba. El tablero sigue prendido para poder
    # ver como el sistema vuelve a levantarse (se apaga con: tablero off).
    bash ~/go2-lab/g1/run_g1.sh stop
    sudo docker exec jetson pkill -f \
        "mobility_authority.py|stand_hold.py|go_to.py|detector.py|agent.py" \
        2>/dev/null
    sleep 2
    echo "robot: $(pgrep -f 'g1_robot.p[y]' | wc -l) instancias vivas"
    echo "GPU:   $(nvidia-smi --query-gpu=memory.used --format=csv,noheader)"
    echo "el tablero sigue prendido (apagarlo: bash run_demo.sh tablero off)"
    ;;

down)
    # La autoridad y stand_hold pertenecen al robot a bordo: siguen activos
    # mientras el robot esté de pie. Sólo se detienen tareas y percepción.
    sudo docker exec jetson pkill -f "go_to.py|detector.py|agent.py"
    echo "misión detenida (robot, autoridad, stand_hold y tablero siguen)"
    ;;

*) echo "uso: $0 {up|start|freeze|clock|pose [reposo|listo|transporte]|check [authority|stand|walk|goto|clock|all]|mission [texto]|kill|tablero [on|off]|status|down}"; exit 1;;
esac
