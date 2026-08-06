#!/usr/bin/env bash
# Repite el export oficial dentro de la escena oficial de Isaac. El evaluador
# adaptado sólo corrige la forma [1, 80] que el export recurrente no acepta.
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
agile_root="${WBC_AGILE_ROOT:-/home/lucas/go2-lab/WBC-AGILE-official}"
isaaclab_root="${ISAACLAB_ROOT:-/home/lucas/go2-lab/IsaacLab-v2.3.2}"
python_executable="${WBC_AGILE_PYTHON:-/home/lucas/go2-lab/venvs/wbc-agile-v2.3.2/bin/python}"
output_root="${WBC_ISAAC_EXPORT_OUTPUT_ROOT:-/home/lucas/experiments/wbc_agile_export_isaac}"
first_repetition="${1:-1}"
last_repetition="${2:-3}"

policy="$agile_root/agile/data/policy/velocity_height_g1/unitree_g1_velocity_height_recurrent_student.pt"
schedule="$script_dir/wbc_agile_walk_stop.yaml"
evaluator="$agile_root/scripts/eval_single_recurrent.py"
if [[ ! -e "$evaluator" ]]; then
    # La fuente oficial queda intacta. La copia hace visible y reversible la
    # única adaptación necesaria para evaluar una policy recurrente exportada.
    cp "$agile_root/scripts/eval.py" "$evaluator"
    patch "$evaluator" <"$script_dir/agile_single_recurrent_eval.patch"
fi

export PYTHONPATH="$agile_root/agile/algorithms/rsl_rl${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONEXE="$python_executable"

for repetition in $(seq "$first_repetition" "$last_repetition"); do
    run_dir="$output_root/run_$repetition"
    if [[ -e "$run_dir" ]]; then
        echo "La corrida ya existe y no se sobrescribirá: $run_dir" >&2
        exit 1
    fi
    mkdir -p "$run_dir"
    echo "INICIO $repetition $(date --iso-8601=seconds)"
    (
        cd "$agile_root"
        "$isaaclab_root/_isaac_sim/python.sh" \
            "$evaluator" \
            --task Velocity-Height-G1-Distillation-Recurrent-v0 \
            --num_envs 1 \
            --checkpoint "$policy" \
            --run_evaluation \
            --save_trajectories \
            --eval_config "$schedule" \
            --metrics_file "$run_dir/metrics.json" \
            --headless
    ) >"$run_dir/launcher.log" 2>&1
    "$python_executable" "$script_dir/analyze_wbc_agile_eval.py" \
        walk_stop "$run_dir/trajectories/episode_000.parquet" \
        >"$run_dir/analysis.json"
    echo "FIN $repetition $(date --iso-8601=seconds)"
done
