#!/usr/bin/env bash
# Ejecuta el archivo desplegable con el adaptador y el cuerpo oficiales. Esta
# referencia separa errores de nuestra integración de los propios de la policy.
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
agile_root="${WBC_AGILE_ROOT:-/home/lucas/go2-lab/WBC-AGILE-official}"
unitree_root="${UNITREE_MUJOCO_ROOT:-/home/lucas/go2-lab/unitree_mujoco-official}"
python_executable="${WBC_AGILE_PYTHON:-/home/lucas/go2-lab/venvs/wbc-agile-v2.3.2/bin/python}"
output_root="${WBC_MUJOCO_OUTPUT_ROOT:-/home/lucas/experiments/wbc_agile_sim2mujoco_export}"

policy="$agile_root/agile/data/policy/velocity_height_g1/unitree_g1_velocity_height_recurrent_student.pt"
descriptor="$agile_root/agile/data/policy/velocity_height_g1/unitree_g1_velocity_height_recurrent_student.yaml"
model="$unitree_root/unitree_robots/g1/scene_29dof.xml"
schedule="$script_dir/wbc_agile_walk_stop.yaml"

mkdir -p "$output_root"
for repetition in 1 2 3; do
    run_dir="$output_root/run_$repetition"
    echo "INICIO $repetition $(date --iso-8601=seconds)"
    (
        cd "$agile_root"
        PYTHONNOUSERSITE=1 "$python_executable" scripts/sim2mujoco_eval.py \
            --checkpoint "$policy" \
            --config "$descriptor" \
            --mjcf "$model" \
            --eval-config "$schedule" \
            --save-data \
            --no-viewer \
            --no-real-time \
            --device cpu \
            --output-dir "$run_dir"
    ) >"$output_root/run_$repetition.log" 2>&1
    trajectory="$(find "$run_dir" -type f -name 'episode_000.parquet' -print -quit)"
    if [[ -z "$trajectory" ]]; then
        echo "No se guardó la trayectoria de la corrida $repetition." >&2
        exit 1
    fi
    "$python_executable" "$script_dir/analyze_agile_sim2mujoco.py" \
        "$trajectory" >"$output_root/run_$repetition.json"
    echo "FIN $repetition $(date --iso-8601=seconds)"
done
