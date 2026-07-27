#!/usr/bin/env bash
# Inyecta imperfecciones de red en el enlace jetson <-> server, para probar el
# sistema contra una red parecida a la real en vez de una red perfecta.
#
# Uso (en la VM):  sudo bash degrade_network.sh {clean|wifi|wifi-bad|outage|show}
#
# Como funciona: usa "tc netem", la herramienta de Linux que degrada el trafico
# de una interfaz a proposito. Se aplica sobre la salida del contenedor server,
# asi que el retardo configurado se siente como tiempo de ida y vuelta. NO se
# aplica del lado jetson porque ese contenedor comparte la red de la VM y
# degradarla cortaria tambien nuestro propio ssh.
#
# Los perfiles son aproximaciones de situaciones reales:
#   wifi      una red de planta razonable, robot cerca del punto de acceso
#   wifi-bad  robot lejos, con interferencia y competencia por el canal
#   outage    el enlace se cae (el robot queda solo)
set -euo pipefail

CONTAINER=server
IFACE=eth0
PROFILE="${1:-show}"

apply() {
    docker exec "$CONTAINER" tc qdisc del dev "$IFACE" root 2>/dev/null || true
    if [ -n "$1" ]; then
        # shellcheck disable=SC2086
        docker exec "$CONTAINER" tc qdisc add dev "$IFACE" root netem $1
    fi
}

case "$PROFILE" in
    clean)
        apply ""
        echo "red limpia (sin retardo ni perdida)";;
    wifi)
        apply "delay 20ms 5ms distribution normal loss 0.2%"
        echo "wifi normal: ~20 ms de ida y vuelta, variacion 5 ms, 0.2% de perdida";;
    wifi-bad)
        apply "delay 80ms 40ms distribution normal loss 3%"
        echo "wifi malo: ~80 ms, variacion 40 ms, 3% de perdida";;
    outage)
        apply "loss 100%"
        echo "ENLACE CORTADO (todo se pierde)";;
    show)
        echo -n "estado actual: "
        docker exec "$CONTAINER" tc qdisc show dev "$IFACE" | head -1;;
    *)
        echo "uso: $0 {clean|wifi|wifi-bad|outage|show}"; exit 1;;
esac
