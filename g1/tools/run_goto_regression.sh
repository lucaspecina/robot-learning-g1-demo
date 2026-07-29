#!/usr/bin/env bash
# Repite navegación después de cambiar su criterio de llegada.
set -u

g1_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
output_root="${GOTO_OUTPUT_ROOT:-$HOME/experiments/goto}"
campaign_name="${1:-$(date +%Y%m%d_%H%M%S)}"
repetitions="${2:-3}"
campaign_dir="$output_root/$campaign_name"
mkdir -p "$campaign_dir"

failures=0
for repetition in $(seq 1 "$repetitions"); do
    log_path="$campaign_dir/goto_run_$repetition.log"
    echo "INICIO goto $repetition $(date --iso-8601=seconds)" \
        | tee -a "$campaign_dir/summary.log"
    if bash "$g1_root/run_demo.sh" check goto >"$log_path" 2>&1; then
        result="PASA"
    else
        result="FALLA"
        failures=$((failures + 1))
    fi
    detail="$(grep -E 'PASA:|FALLA:' "$log_path" | tail -1 | sed 's/^[[:space:]]*//')"
    echo "$result goto $repetition: $detail" \
        | tee -a "$campaign_dir/summary.log"
done

echo "Fallas: $failures de $repetitions" | tee -a "$campaign_dir/summary.log"
exit "$((failures > 0))"
