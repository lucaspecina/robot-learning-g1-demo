#!/usr/bin/env bash
# Repite quietud y caminata después de un cambio en locomoción. Continúa las
# seis corridas aunque una falle para distinguir una tendencia de un accidente.
set -u

g1_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
output_root="${INTEGRATION_REGRESSION_OUTPUT:-$HOME/experiments/integration_regression}"
campaign_name="${1:-$(date +%Y%m%d_%H%M%S)}"
campaign_dir="$output_root/$campaign_name"
mkdir -p "$campaign_dir"

failures=0
for repetition in 1 2 3; do
    for scenario in stand walk; do
        log_path="$campaign_dir/${scenario}_run_${repetition}.log"
        echo "INICIO $scenario $repetition $(date --iso-8601=seconds)" | tee -a "$campaign_dir/summary.log"
        # Cada medición debe probar la policy activa; congelado parece perfecto
        # porque el simulador vuelve a escribir la misma pose en cada cuadro.
        bash "$g1_root/run_demo.sh" start >/dev/null
        if bash "$g1_root/run_demo.sh" check "$scenario" >"$log_path" 2>&1; then
            result="PASA"
        else
            result="FALLA"
            failures=$((failures + 1))
        fi
        detail="$(grep -E 'PASA:|FALLA:' "$log_path" | tail -1 | sed 's/^[[:space:]]*//')"
        echo "$result $scenario $repetition: $detail" | tee -a "$campaign_dir/summary.log"
    done
done

echo "Fallas: $failures de 6" | tee -a "$campaign_dir/summary.log"
exit "$((failures > 0))"
