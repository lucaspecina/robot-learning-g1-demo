#!/usr/bin/env bash
# Lanza el robot G1 simulado, uno solo, desacoplado del ssh.
#
# Uso (en la VM):
#   bash run_g1.sh policy [modelo] [carga_kg] [extra...]
#       modelo    12dof (el cuerpo que la policy conoce) | 29dof (con brazos)
#       carga_kg  masa extra en las manos, para medir cuanto tolera
#       extra     --camera para montar la camara de la cabeza
#   bash run_g1.sh stand [modelo]   -> sin policy, solo sostiene la pose
#   bash run_g1.sh stop
#   bash run_g1.sh status
#
# Ejemplos:
#   bash run_g1.sh policy 29dof              robot con brazos, sin carga
#   bash run_g1.sh policy 29dof 1.0          con 1 kg en las manos
#   bash run_g1.sh policy 29dof 0 --camera   con la camara publicando
set -uo pipefail

POLICY=~/go2-lab/unitree_rl_gym/deploy/pre_train/g1/motion.pt
LOG=~/g1.log
MODE="${1:-policy}"
MODEL="${2:-12dof}"
PAYLOAD="${3:-0}"
EXTRA="${4:-}"

case "$MODE" in
    stop|status) ;;
    *)
        pkill -9 -f "g1_robot.p[y]" 2>/dev/null
        sleep 3
        ;;
esac

case "$MODE" in
    policy) ARGS="--policy $POLICY --model $MODEL --payload_kg $PAYLOAD $EXTRA" ;;
    stand)  ARGS="--model $MODEL --payload_kg $PAYLOAD $EXTRA" ;;
    stop)
        pkill -9 -f "g1_robot.p[y]" && echo "detenido" || echo "no habia nada"
        exit 0;;
    status)
        pgrep -f "g1_robot.p[y]" >/dev/null && echo "corriendo" || echo "detenido"
        tail -c 500 "$LOG" | tr '\r' '\n' | tail -4
        exit 0;;
    *) echo "uso: $0 {policy|stand|stop|status} [modelo] [carga_kg] [extra]"; exit 1;;
esac

# Isaac exige este flag para renderizar camaras; sin el, crear una es un error.
case "$EXTRA" in
    *--camera*) ARGS="$ARGS --enable_cameras" ;;
esac

# Modo de video: con "--visible" transmite la escena al cliente de Isaac; por
# defecto corre sin dibujar la ventana (mas rapido, para pruebas automaticas).
MODO_VIDEO="--headless"
case "$EXTRA" in
    *--visible*)
        IP_PUBLICA=$(curl -s --max-time 5 https://api.ipify.org)
        MODO_VIDEO="--livestream 2 --/app/livestream/publicEndpointAddress=$IP_PUBLICA"
        ARGS="${ARGS//--visible/}"
        echo "video: transmitiendo a $IP_PUBLICA (conectate con el cliente de Isaac)"
        ;;
esac

cd ~/go2-lab/g1 || exit 1
source ~/go2-lab/isaac_ros_env.sh
setsid nohup ~/go2-lab/IsaacLab/isaaclab.sh -p g1_robot.py $MODO_VIDEO $ARGS \
    > "$LOG" 2>&1 < /dev/null &
echo "lanzado: modo=$MODE modelo=$MODEL carga=${PAYLOAD}kg ${EXTRA}"
echo "seguir con: bash run_g1.sh status"
