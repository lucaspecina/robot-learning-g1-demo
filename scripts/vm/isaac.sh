#!/usr/bin/env bash
# Manejo de Isaac Sim en modo streaming.
# Uso: bash isaac.sh {start|stop|status|log|check}
#
# Rescatado del disco de la VM el 10-ago-2026; no estaba versionado.
# Cambio respecto del original: la IP pública se consulta en vivo en lugar de
# estar escrita a mano, porque cambia en cada encendido y porque este script
# tiene que servir también en la máquina nueva.

IP=$(curl -s --max-time 5 https://api.ipify.org)
[ -z "$IP" ] && IP=$(curl -s --max-time 5 https://ifconfig.me)
if [ -z "$IP" ]; then echo "ERROR: no pude averiguar la IP pública" >&2; exit 1; fi

alive() { pgrep -f "isaacsim.exp.full.streamin[g]" >/dev/null; }
ready() { ss -tln | grep -q 49100; }

case "$1" in
  start)
    if alive; then echo "ya esta corriendo"; ready && echo "y LISTO para conectar"; exit 0; fi
    cd /opt/IsaacSim || exit 1
    setsid nohup env ACCEPT_EULA=Y PRIVACY_CONSENT=Y ./isaac-sim.streaming.sh \
      --/app/livestream/publicEndpointAddress="$IP" \
      > ~/isaac_stream.log 2>&1 < /dev/null &
    echo "Isaac lanzado (desacoplado — Ctrl+C aca no lo toca)."
    echo "Esperando streaming (1-3 min)..."
    for i in $(seq 1 120); do
      if ready; then echo; echo "*** LISTO: conectate con el cliente a $IP ***"; exit 0; fi
      if ! alive; then echo; echo "*** MURIO al arrancar. Ver: bash isaac.sh log ***"; exit 1; fi
      sleep 3; printf .
    done
    echo; echo "No levanto en 6 min. Ver: bash isaac.sh log"; exit 1;;
  stop)   pkill -f "isaacsim.exp.full.streamin[g]" && echo "detenido" || echo "no habia nada corriendo";;
  status) alive && echo "proceso: vivo" || echo "proceso: muerto"
          ready && echo "streaming: LISTO para conectar" || echo "streaming: no listo";;
  log)    tail -20 ~/isaac_stream.log;;
  check)
    if ! alive; then echo "proceso: MUERTO -> relanzar: bash isaac.sh start"; exit 1; fi
    PID=$(pgrep -f "isaacsim.exp.full.streamin[g]" | head -1)
    CPU=$(ps -o %cpu= -p "$PID" | cut -d. -f1 | tr -d " ")
    GPU=$(nvidia-smi --query-gpu=utilization.gpu --format=csv,noheader,nounits)
    ERR=$(grep -ciE "\[error\]" ~/isaac_stream.log)
    echo "proceso: vivo | CPU Isaac: ${CPU}% (normal: 100-150) | GPU: ${GPU}% | errores: ${ERR}"
    # Heuristica: la GPU manda. Cargando = GPU parada; renderizando = GPU activa.
    # (la CPU esta SIEMPRE alta en esta app: render+stream; no discrimina nada)
    if [ "${GPU:-0}" -ge 10 ]; then
      echo "VEREDICTO: RENDERIZANDO normal. Pantalla negra? 1) tecla F  2) reconectar el cliente (stream congelado)"
    elif [ "${CPU:-0}" -gt 60 ]; then
      echo "VEREDICTO: CARGANDO (GPU parada, CPU alta) -> esperar"
    else
      echo "VEREDICTO: QUIETO -> si hay sintomas, reiniciar: stop && start"
    fi;;
  *)      echo "uso: $0 {start|stop|status|log|check}";;
esac
