#!/usr/bin/env bash
# Lanza el robot G1 simulado, uno solo, desacoplado del ssh.
#
# Uso (en la VM):
#   bash run_g1.sh wbc [modelo] [carga_kg] [extra...]
#       modelo    12dof (el cuerpo que la policy conoce) | 29dof (con brazos)
#       carga_kg  masa extra en las manos, para medir cuanto tolera
#       extra     --camera para montar la camara de la cabeza
#   bash run_g1.sh legacy [modelo]  -> conserva la locomoción anterior
#   bash run_g1.sh stand [modelo]   -> sin policy, solo sostiene la pose
#   bash run_g1.sh stop
#   bash run_g1.sh status
#
# Ejemplos:
#   bash run_g1.sh wbc 29dof              robot con brazos, sin carga
#   bash run_g1.sh wbc 29dof 1.0          con 1 kg en las manos
#   bash run_g1.sh wbc 29dof 0 --camera   con la camara publicando
set -uo pipefail

# Dos lanzadores casi simultáneos podían completar ambos su limpieza antes de
# que cualquiera registrara el nuevo proceso. El bloqueo serializa esa zona
# crítica; el robot queda desprendido y el archivo se libera al terminar.
exec 9>/tmp/g1_robot_launcher.lock
if ! flock -n 9; then
    echo "ERROR: ya hay otro lanzador modificando el robot" >&2
    exit 2
fi

WBC_ROOT=~/go2-lab/WBC-AGILE
WBC_COMMIT=7259792cf10803aab814d101134d493d24c8f22f
WBC_POLICY=$WBC_ROOT/agile/data/policy/velocity_height_g1/unitree_g1_velocity_height_recurrent_student.pt
WBC_DESCRIPTOR=$WBC_ROOT/agile/data/policy/velocity_height_g1/unitree_g1_velocity_height_recurrent_student.yaml
LEGACY_POLICY=~/go2-lab/unitree_rl_gym/deploy/pre_train/g1/motion.pt
LOG=~/g1.log
MODE="${1:-wbc}"
MODEL="${2:-29dof}"
PAYLOAD="${3:-0}"
EXTRA="${4:-}"

# Matar TODAS las instancias previas y verificar que murieron de verdad.
# Un solo pkill no alcanza: una instancia arrancando puede escaparse y quedar
# de zombi, compitiendo por la GPU y hablando por los mismos topics de ROS
# (nos paso: tres robots a la vez, RTF degradado y caidas inexplicables).
kill_all_robots() {
    for _ in 1 2 3 4 5; do
        pgrep -f "g1_robot.p[y]" >/dev/null || return 0
        pkill -9 -f "g1_robot.p[y]" 2>/dev/null
        sleep 2
    done
    echo "AVISO: no pude matar todas las instancias previas:" >&2
    pgrep -af "g1_robot.p[y]" >&2
    return 1
}

case "$MODE" in
    stop|status) ;;
    *)
        kill_all_robots
        sleep 1
        ;;
esac

case "$MODE" in
    wbc)
        ACTUAL_COMMIT=$(git -C "$WBC_ROOT" rev-parse HEAD 2>/dev/null || true)
        if [ "$ACTUAL_COMMIT" != "$WBC_COMMIT" ]; then
            echo "ERROR: WBC-AGILE no está en la versión verificada." >&2
            echo "esperada: $WBC_COMMIT" >&2
            echo "actual:   ${ACTUAL_COMMIT:-no encontrada}" >&2
            exit 1
        fi
        ARGS="--locomotion wbc --policy $WBC_POLICY --policy-descriptor $WBC_DESCRIPTOR --model 29dof --payload_kg $PAYLOAD $EXTRA"
        ;;
    legacy|policy)
        ARGS="--locomotion legacy --policy $LEGACY_POLICY --model $MODEL --payload_kg $PAYLOAD $EXTRA"
        ;;
    stand)
        ARGS="--locomotion stand --model $MODEL --payload_kg $PAYLOAD $EXTRA"
        ;;
    stop)
        if kill_all_robots; then
            echo "detenido (todas las instancias)"
            exit 0
        fi
        echo "quedaron instancias vivas" >&2
        exit 1;;
    status)
        pgrep -f "g1_robot.p[y]" >/dev/null && echo "corriendo" || echo "detenido"
        tail -c 500 "$LOG" | tr '\r' '\n' | tail -4
        exit 0;;
    *) echo "uso: $0 {wbc|legacy|stand|stop|status} [modelo] [carga_kg] [extra]"; exit 1;;
esac

# Isaac exige este flag para renderizar camaras; sin el, crear una es un error.
case "$EXTRA" in
    *--camera*|*--lidar*) ARGS="$ARGS --enable_cameras" ;;
esac

# Modo de video: con "--visible" transmite la escena al cliente de Isaac; por
# defecto corre sin dibujar la ventana (mas rapido, para pruebas automaticas).
MODO_VIDEO="--headless"
case "$EXTRA" in
    *--visible*)
        IP_PUBLICA=$(curl -s --max-time 5 https://api.ipify.org)
        export PUBLIC_IP="$IP_PUBLICA"
        MODO_VIDEO="--livestream 1"
        ARGS="${ARGS//--visible/}"
        echo "video: transmitiendo a $IP_PUBLICA (conectate con el cliente de Isaac)"
        ;;
esac

cd ~/go2-lab/g1 || exit 1
source ~/go2-lab/isaac_ros_env.sh
export PYTHONPATH="$WBC_ROOT${PYTHONPATH:+:$PYTHONPATH}"
# Vaciar el log antes de desprender el proceso impide que un lanzador lea el
# `LISTO` de una corrida anterior si Isaac todavía no llegó a iniciar.
: > "$LOG"
setsid nohup ~/go2-lab/IsaacLab/isaaclab.sh -p g1_robot.py $MODO_VIDEO $ARGS \
    > "$LOG" 2>&1 < /dev/null 9>&- &
echo "lanzado: modo=$MODE modelo=$MODEL carga=${PAYLOAD}kg ${EXTRA}"
echo "seguir con: bash run_g1.sh status"
