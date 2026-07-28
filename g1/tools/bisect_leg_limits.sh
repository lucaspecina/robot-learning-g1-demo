#!/bin/bash
# Biseccion: ¿que tira al robot, el tope de FUERZA o el de VELOCIDAD que le
# pusimos a las piernas? Tres corridas sobre el codigo de referencia que
# funciona (en ~/g1_ayer), cambiando UNA variable por vez.
#
# Uso (en la VM):  bash bisect_leg_limits.sh
cd ~/g1_ayer || exit 1
source ~/go2-lab/isaac_ros_env.sh

run_case() {
    local LABEL="$1"; shift
    echo "[$(date +%H:%M:%S)] $LABEL  args: $*"
    pkill -9 -f "g1_robot.p[y]" 2>/dev/null; sleep 3
    setsid nohup ~/go2-lab/IsaacLab/isaaclab.sh -p g1_robot.py --headless \
        --policy ~/go2-lab/unitree_rl_gym/deploy/pre_train/g1/motion.pt \
        --model 29dof "$@" > ~/g1_ayer.log 2>&1 < /dev/null &
    local i
    for i in $(seq 1 60); do
        tr '\r' '\n' < ~/g1_ayer.log | grep -qE "RTF|CONGELADO" && break
        pgrep -f "g1_robot.p[y]" >/dev/null || { echo "  -> NO ARRANCO"; return; }
        sleep 5
    done
    # Si quedo congelado, la prueba no midio la policy: es un error de prueba,
    # no un "de pie". (Ya nos comimos ese falso positivo una vez.)
    if tr '\r' '\n' < ~/g1_ayer.log | grep -q CONGELADO; then
        echo "  -> ERROR DE PRUEBA: quedo congelado (falta --free)"; return
    fi
    sleep 45
    local SAMPLES MIN
    SAMPLES=$(tr '\r' '\n' < ~/g1_ayer.log | grep -oP "altura \K[0-9.]+" | tail -5 | tr '\n' ' ')
    MIN=$(echo "$SAMPLES" | tr ' ' '\n' | sort -n | head -1)
    if awk "BEGIN{exit !(${MIN:-0} > 0.6)}" 2>/dev/null; then
        echo "  -> DE PIE   (min $MIN | $SAMPLES)"
    else
        echo "  -> SE CAYO  (min $MIN | $SAMPLES)"
    fi
}

echo "=== ¿FUERZA o VELOCIDAD? ==="
run_case "A control todo-stock"   --free --leg_effort stock --leg_velocity stock
run_case "B solo velocidad urdf"  --free --leg_effort stock --leg_velocity urdf
run_case "C solo fuerza urdf"     --free --leg_effort urdf  --leg_velocity stock
pkill -9 -f "g1_robot.p[y]" 2>/dev/null
echo "=== FIN ==="
