#!/usr/bin/env bash
# Levanta el sistema completo de la demo, en orden, y deja todo listo para mirar.
#
# Uso (en la VM):
#   bash run_demo.sh up        arranca robot + skills + agente + tablero
#   bash run_demo.sh mission   le da la mision de la demo
#   bash run_demo.sh status    como viene todo
#   bash run_demo.sh down      apaga las capas de arriba (no el robot)
#
# Para mirarlo desde tu maquina, dos ventanas:
#   - el tablero:  ssh -L 8080:localhost:8080 lucas@<IP>  -> http://localhost:8080
#   - el robot:    cliente de Isaac apuntando a la IP de la VM
set -uo pipefail

D=/workspace/g1          # ruta dentro del contenedor jetson
ROS="source /opt/ros/jazzy/setup.bash"

lanzar() {   # nombre archivo_log script
    sudo docker exec -d jetson bash -c "$ROS && python3 $D/$3 > /tmp/$2 2>&1"
    echo "   $1"
}

case "${1:-up}" in
up)
    echo ">> El robot (Isaac, ~1 min de arranque)..."
    bash ~/go2-lab/g1/run_g1.sh policy 29dof 0 "--camera --scene --visible" | tail -1

    echo ">> Esperando a que el robot este en pie..."
    until tr '\r' '\n' < ~/g1.log 2>/dev/null | grep -q "RTF"; do
        pgrep -f "g1_robot.p[y]" >/dev/null || { echo "   el robot no arranco"; exit 1; }
        sleep 5
    done
    echo "   listo"

    echo ">> Las capas de arriba, en la jetson:"
    sudo docker exec jetson pkill -f "go_to.py|detector.py|agent.py|dashboard.py" 2>/dev/null
    sleep 2
    lanzar "navegacion"  goto.log      skills/go_to.py
    lanzar "percepcion"  detector.log  skills/detector.py
    lanzar "agente"      agent.log     agent/agent.py
    lanzar "tablero"     dashboard.log dashboard/dashboard.py
    sleep 6

    echo ""
    echo "TODO ARRIBA. Para mirarlo:"
    echo "  tablero:  ssh -L 8080:localhost:8080 lucas@\$(curl -s https://api.ipify.org)"
    echo "            y abrir http://localhost:8080"
    echo "  robot:    cliente de Isaac -> \$(curl -s https://api.ipify.org)"
    echo ""
    echo "Darle la mision:  bash run_demo.sh mission"
    ;;

mission)
    MISION="${2:-fijate la hora en el reloj y llevale la botella a quien corresponda}"
    sudo docker exec jetson bash -c \
        "$ROS && timeout 8 ros2 topic pub --once /g1/mission std_msgs/msg/String \"{data: $MISION}\"" \
        >/dev/null 2>&1
    echo "mision enviada: $MISION"
    echo "seguila en el tablero, o con: bash run_demo.sh status"
    ;;

status)
    echo "== el robot =="
    pgrep -f "g1_robot.p[y]" >/dev/null && echo "  corriendo" || echo "  detenido"
    tr '\r' '\n' < ~/g1.log 2>/dev/null | grep RTF | tail -1 | sed 's/^/  /'
    echo "== las capas de arriba =="
    for p in go_to detector agent dashboard; do
        if sudo docker exec jetson pgrep -f "$p.py" >/dev/null 2>&1; then
            echo "  $p: corriendo"
        else
            echo "  $p: DETENIDO"
        fi
    done
    echo "== la mision =="
    sudo docker exec jetson tail -6 /tmp/agent.log 2>/dev/null \
        | sed 's/\[INFO\] \[[0-9.]*\] \[agent\]: /  /'
    ;;

down)
    sudo docker exec jetson pkill -f "go_to.py|detector.py|agent.py|dashboard.py"
    echo "capas de arriba detenidas (el robot sigue)"
    ;;

*) echo "uso: $0 {up|mission [texto]|status|down}"; exit 1;;
esac
