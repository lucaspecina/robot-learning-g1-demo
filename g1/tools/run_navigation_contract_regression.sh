#!/usr/bin/env bash
# Prueba las terminaciones peligrosas sin mezclar el resultado con la física.
# Congela el robot, induce cada falla y exige que la autoridad vuelva a STAND.
set -euo pipefail

g1_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
ros_setup="source /opt/ros/jazzy/setup.bash"
check="/workspace/g1/tools/check_navigation_action.py"
failures=0

stop_navigation() {
    sudo docker exec jetson pkill -f '[g]o_to.py' >/dev/null 2>&1 || true
    sleep 1
}

start_navigation() {
    local environment="$1"
    local log_path="$2"
    sudo docker exec -d jetson bash -lc \
        "$ros_setup; $environment python3 /workspace/g1/skills/go_to.py \
        >$log_path 2>&1"
    sleep 2
}

restore_layers() {
    stop_navigation
    bash "$g1_root/run_demo.sh" layers >/dev/null 2>&1 || true
}
trap restore_layers EXIT

run_expected_abort() {
    local name="$1"
    local environment="$2"
    local expected_text="$3"
    local log_path="/tmp/navigation_${name}.log"

    stop_navigation
    start_navigation "$environment" "$log_path"
    if sudo docker exec jetson bash -lc \
        "$ros_setup; python3 $check --x 1 --y 0 --yaw 0 \
        --expect aborted --timeout 15" \
        | tee "/tmp/${name}_client.log" \
        && grep -F "$expected_text" "/tmp/${name}_client.log" >/dev/null
    then
        echo "PASA: $name terminó abortado y volvió a STAND"
    else
        echo "FALLA: $name no cumplió el contrato"
        sudo docker exec jetson tail -40 "$log_path" || true
        failures=$((failures + 1))
    fi
}

echo ">> Congelando el cuerpo para aislar el contrato de la física"
bash "$g1_root/run_demo.sh" freeze >/dev/null
sleep 2

run_expected_abort \
    "sin_progreso" \
    "NAV_PROGRESS_TIMEOUT_S=3 NAV_EXECUTION_TIMEOUT_S=30" \
    "navegación sin progreso"

run_expected_abort \
    "plazo_vencido" \
    "NAV_PROGRESS_TIMEOUT_S=20 NAV_EXECUTION_TIMEOUT_S=2" \
    "venció el plazo total"

echo ">> Matando el servidor durante un objetivo activo"
stop_navigation
start_navigation \
    "NAV_PROGRESS_TIMEOUT_S=30 NAV_EXECUTION_TIMEOUT_S=60" \
    "/tmp/navigation_process_dead.log"
sudo docker exec -d jetson bash -lc \
    "$ros_setup; python3 $check --x 1 --y 0 --yaw 0 --timeout 10 \
    >/tmp/process_dead_client.log 2>&1"
sleep 1
stop_navigation
sleep 2

owner="$(
    sudo docker exec jetson bash -lc \
        "$ros_setup; timeout 5 ros2 topic echo --once --field data \
        /g1/mobility/status" \
        2>/dev/null \
        | python3 -c 'import json,sys; print(json.loads(sys.stdin.readline())["owner"])'
)"
if [ "$owner" = "stand" ]; then
    echo "PASA: proceso muerto devolvió la autoridad a STAND"
else
    echo "FALLA: proceso muerto dejó la autoridad en $owner"
    failures=$((failures + 1))
fi

echo "Fallas: $failures de 3"
exit "$((failures > 0))"
