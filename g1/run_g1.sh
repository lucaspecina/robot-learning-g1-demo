#!/usr/bin/env bash
# Lanza el robot G1 simulado, uno solo, desacoplado del ssh.
#
# Uso (en la VM):
#   bash run_g1.sh policy     -> con la policy de locomocion RL
#   bash run_g1.sh stand      -> solo sosteniendo la pose (diagnostico)
#   bash run_g1.sh stop       -> detenerlo
#   bash run_g1.sh status     -> ver como va
set -uo pipefail

POLICY=~/go2-lab/unitree_rl_gym/deploy/pre_train/g1/motion.pt
LOG=~/g1.log
MODE="${1:-policy}"

case "$MODE" in
    stop|status) ;;
    *)
        pkill -9 -f "g1_robot.p[y]" 2>/dev/null
        sleep 3
        ;;
esac

case "$MODE" in
    policy) ARGS="--policy $POLICY" ;;
    stand)  ARGS="" ;;
    stop)
        pkill -9 -f "g1_robot.p[y]" && echo "detenido" || echo "no habia nada"
        exit 0;;
    status)
        pgrep -f "g1_robot.p[y]" >/dev/null && echo "corriendo" || echo "detenido"
        tail -c 400 "$LOG" | tr '\r' '\n' | tail -3
        exit 0;;
    *) echo "uso: $0 {policy|stand|stop|status}"; exit 1;;
esac

cd ~/go2-lab/g1 || exit 1
source ~/go2-lab/isaac_ros_env.sh
setsid nohup ~/go2-lab/IsaacLab/isaaclab.sh -p g1_robot.py --headless $ARGS \
    > "$LOG" 2>&1 < /dev/null &
echo "lanzado en modo $MODE (arranque ~1 min). Seguir con: bash run_g1.sh status"
