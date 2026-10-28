#!/usr/bin/env bash
# Ejecuta una evaluación oficial intacta de WBC-AGILE. Las rutas se pueden
# reemplazar por variables de entorno porque la prueba debe ser reproducible
# sin mezclar su configuración con la del robot de la demo.
set -euo pipefail

if [[ $# -ne 2 ]]; then
    echo "Uso: $0 {stand|walk_stop|watch} NOMBRE_DE_CORRIDA" >&2
    exit 2
fi

scenario="$1"
run_name="$2"
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
wbc_root="${WBC_AGILE_ROOT:-/home/lucas/go2-lab/WBC-AGILE}"
isaaclab_root="${ISAACLAB_ROOT:-/home/lucas/go2-lab/IsaacLab}"
python_executable="${WBC_AGILE_PYTHON:-/home/lucas/go2-lab/venvs/wbc-agile/bin/python}"
output_root="${WBC_EVAL_OUTPUT_ROOT:-/home/lucas/experiments/wbc_agile}"
checkpoint="$wbc_root/agile/data/policy/velocity_height_g1/unitree_g1_velocity_height_recurrent_student_checkpoint.pt"
livestream_public_ip="${WBC_LIVESTREAM_PUBLIC_IP:-}"

case "$scenario" in
    stand)
        config_path="$script_dir/wbc_agile_stand.yaml"
        ;;
    walk_stop)
        config_path="$script_dir/wbc_agile_walk_stop.yaml"
        ;;
    watch)
        config_path="$script_dir/wbc_agile_watch.yaml"
        ;;
    *)
        echo "Escenario desconocido: $scenario" >&2
        exit 2
        ;;
esac

run_dir="$output_root/$run_name"
if [[ -e "$run_dir" ]]; then
    echo "La corrida ya existe y no se sobrescribirá: $run_dir" >&2
    exit 1
fi

if pgrep -f "^$python_executable scripts/eval.py" >/dev/null; then
    echo "Ya hay una evaluación de WBC-AGILE usando la GPU." >&2
    exit 1
fi

mkdir -p "$run_dir"
cd "$wbc_root"

# La transmisión se reserva para observación humana: por defecto las
# mediciones no renderizan para evitar que la carga gráfica altere su ritmo.
launcher_args=(--headless)
if [[ -n "$livestream_public_ip" ]]; then
    export PUBLIC_IP="$livestream_public_ip"
    launcher_args=(--livestream 1)
fi

# El repositorio trae su propia versión del ejecutor neuronal. Se prioriza
# explícitamente porque una versión global distinta carga el archivo pero
# interpreta mal su estructura y produce una falsa falla de la prueba.
export PYTHONPATH="$wbc_root/agile/algorithms/rsl_rl${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONEXE="$python_executable"

exec "$isaaclab_root/_isaac_sim/python.sh" scripts/eval.py \
    --task Velocity-Height-G1-Distillation-Recurrent-v0 \
    --num_envs 1 \
    --checkpoint "$checkpoint" \
    --run_evaluation \
    --save_trajectories \
    --eval_config "$config_path" \
    --metrics_file "$run_dir/metrics.json" \
    "${launcher_args[@]}"
