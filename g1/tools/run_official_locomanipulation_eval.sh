#!/usr/bin/env bash
# Ejecuta la referencia intacta de NVIDIA y guarda sólo mediciones externas.
set -euo pipefail

isaaclab_root="${ISAACLAB_REFERENCE_ROOT:-/home/lucas/go2-lab/IsaacLab-v2.3.2}"
compat_root="${ISAACLAB_REFERENCE_COMPAT_ROOT:-/home/lucas/go2-lab/compat/isaaclab-v2.3.2-numpy1}"
demo_root="${G1_DEMO_ROOT:-/home/lucas/go2-lab/g1}"
task_id="${G1_OFFICIAL_LOCOMANIP_TASK:-Isaac-PickPlace-Locomanipulation-G1-Abs-v0}"
duration_s="${G1_OFFICIAL_LOCOMANIP_DURATION_S:-10}"
repetitions="${G1_OFFICIAL_LOCOMANIP_REPETITIONS:-3}"
device="${G1_OFFICIAL_LOCOMANIP_DEVICE:-cuda:0}"
output_dir="${G1_OFFICIAL_LOCOMANIP_OUTPUT_DIR:-${demo_root}/logs/official_locomanipulation}"
stamp="$(date -u +%Y%m%dT%H%M%SZ)"
log_path="${output_dir}/${stamp}.log"
report_path="${output_dir}/${stamp}.json"
reference_python_paths="${compat_root}:${isaaclab_root}/source/isaaclab:${isaaclab_root}/source/isaaclab_tasks:${isaaclab_root}/source/isaaclab_assets"

if [[ ! -x "${isaaclab_root}/isaaclab.sh" ]]; then
    echo "ERROR: no existe la instalación fijada de Isaac Lab: ${isaaclab_root}" >&2
    exit 2
fi
if [[ ! -f "${demo_root}/tools/evaluate_official_locomanipulation.py" ]]; then
    echo "ERROR: falta el evaluador: ${demo_root}/tools/evaluate_official_locomanipulation.py" >&2
    exit 2
fi
if [[ ! -f "${compat_root}/numpy/__init__.py" ]]; then
    echo "ERROR: falta la capa aislada NumPy 1 para Pinocchio: ${compat_root}" >&2
    exit 2
fi
if pgrep -f 'g1_robot.py|zero_agent.py.*Locomanipulation|evaluate_official_locomanipulation.py' >/dev/null; then
    echo "ERROR: ya hay una simulación del G1 usando la referencia o la demo." >&2
    exit 3
fi

mkdir -p "${output_dir}"
echo "Referencia: Isaac Lab v2.3.2 sin cambios"
echo "Tarea: ${task_id}"
echo "Repeticiones: ${repetitions} de ${duration_s} s simulados"
echo "Dispositivo: ${device}"
echo "Log: ${log_path}"
echo "Informe: ${report_path}"

cd "${isaaclab_root}"
# Hay otra copia editable de Isaac Lab instalada en la VM. Prefijar las rutas
# fijadas evita que el lanzador de v2.3.2 importe silenciosamente la rama main.
export PYTHONPATH="${reference_python_paths}${PYTHONPATH:+:${PYTHONPATH}}"
reference_python="${isaaclab_root}/_isaac_sim/kit/python/bin/python3"
"${reference_python}" -c \
    'import numpy, pinocchio; assert int(numpy.__version__.split(".")[0]) < 2; print(f"NumPy compatible: {numpy.__version__}; Pinocchio: {pinocchio.__version__}")'
set +e
./isaaclab.sh -p "${demo_root}/tools/evaluate_official_locomanipulation.py" \
    --task "${task_id}" \
    --expected-isaaclab-root "${isaaclab_root}" \
    --duration-s "${duration_s}" \
    --repetitions "${repetitions}" \
    --output "${report_path}" \
    --device "${device}" \
    --enable_pinocchio \
    --headless \
    >"${log_path}" 2>&1
exit_code=$?
set -e

if [[ "${exit_code}" -ne 0 ]]; then
    echo "ERROR: la referencia terminó con código ${exit_code}." >&2
    tail -80 "${log_path}" >&2
    exit "${exit_code}"
fi
if [[ ! -s "${report_path}" ]]; then
    echo "ERROR: Isaac terminó sin producir el informe medido." >&2
    tail -80 "${log_path}" >&2
    exit 4
fi

cat "${report_path}"
