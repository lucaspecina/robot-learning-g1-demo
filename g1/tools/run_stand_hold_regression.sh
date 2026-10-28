#!/usr/bin/env bash
# Repite quietud para medir el controlador y ejecuta una caminata final para
# confirmar que el cambio no afectó el resto de la locomoción.
set -u

g1_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
output_root="${STAND_HOLD_OUTPUT_ROOT:-$HOME/experiments/stand_hold}"
campaign_name="${1:-$(date +%Y%m%d_%H%M%S)}"
campaign_dir="$output_root/$campaign_name"
mkdir -p "$campaign_dir"

failures=0
for repetition in 1 2 3; do
    log_path="$campaign_dir/stand_run_$repetition.log"
    echo "INICIO stand $repetition $(date --iso-8601=seconds)" \
        | tee -a "$campaign_dir/summary.log"
    if bash "$g1_root/run_demo.sh" check stand >"$log_path" 2>&1; then
        result="PASA"
    else
        result="FALLA"
        failures=$((failures + 1))
    fi
    detail="$(grep -E 'PASA:|FALLA:' "$log_path" | tail -1 | sed 's/^[[:space:]]*//')"
    echo "$result stand $repetition: $detail" \
        | tee -a "$campaign_dir/summary.log"
done

walk_log="$campaign_dir/walk_run_1.log"
echo "INICIO walk 1 $(date --iso-8601=seconds)" \
    | tee -a "$campaign_dir/summary.log"
if bash "$g1_root/run_demo.sh" check walk >"$walk_log" 2>&1; then
    result="PASA"
else
    result="FALLA"
    failures=$((failures + 1))
fi
detail="$(grep -E 'PASA:|FALLA:' "$walk_log" | tail -1 | sed 's/^[[:space:]]*//')"
echo "$result walk 1: $detail" | tee -a "$campaign_dir/summary.log"

echo "Fallas: $failures de 4" | tee -a "$campaign_dir/summary.log"
exit "$((failures > 0))"
