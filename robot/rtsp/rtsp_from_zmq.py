#!/usr/bin/env python3
"""Transmite por RTSP el video que ya sirve `teleimager-server`, sin tocar la cámara.

`teleimager-server` (el servidor de imágenes de la teleoperación, y el que
alimenta a la ACT durante la tarea) publica cada cuadro de la cámara de la
cabeza como un JPEG suelto por ZMQ (PUB, tcp://*:55555). Cualquier cantidad
de suscriptores puede leerlo a la vez: la sesión de brazos y `wait_tag.py` ya
lo hacen. Este script es un suscriptor más: toma los JPEG y se los pasa a
ffmpeg, que los comprime en H.264 y los publica en un servidor RTSP
(MediaMTX). Así la transmisión convive con la tarea en vez de robarle la
cámara, que admite un solo dueño.

Uso (en el robot, con el Python del entorno `teleoperation`, que tiene pyzmq):

    python3 rtsp_from_zmq.py --rtsp-url rtsp://IP_DEL_SERVIDOR_MEDIAMTX:8554/g1_head_cam

Requiere `teleimager-server` corriendo (`~/g1-task/camera.sh policy`) y
ffmpeg instalado. Ctrl-C para terminar; no deja nada corriendo.
"""
from __future__ import annotations

import argparse
import signal
import subprocess
import sys
import time

import zmq


def build_ffmpeg_cmd(fps: int, bitrate: str, rtsp_url: str, encoder: str) -> list[str]:
    # La entrada es una secuencia de JPEG concatenados (lo que manda teleimager);
    # el demuxer `mjpeg` de ffmpeg la lee tal cual, sin decodificar en Python.
    cmd = [
        "ffmpeg", "-hide_banner", "-loglevel", "warning", "-nostdin",
        "-fflags", "nobuffer", "-flags", "low_delay",
        "-use_wallclock_as_timestamps", "1",
        "-framerate", str(fps), "-f", "mjpeg", "-i", "-",
        "-an", "-pix_fmt", "yuv420p", "-r", str(fps),
    ]
    if encoder == "libx264":
        # ultrafast + zerolatency: que salga ya, antes que perfecto (vigilancia).
        cmd += ["-c:v", "libx264", "-preset", "ultrafast", "-tune", "zerolatency",
                "-b:v", bitrate, "-g", str(fps * 2)]
    else:
        cmd += ["-c:v", encoder, "-b:v", bitrate]
    cmd += ["-f", "rtsp", "-rtsp_transport", "tcp", rtsp_url]
    return cmd


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--zmq", default="tcp://127.0.0.1:55555", help="dónde publica teleimager-server")
    ap.add_argument("--rtsp-url", required=True, help="destino, p. ej. rtsp://HOST:8554/g1_head_cam")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--bitrate", default="1500k")
    ap.add_argument("--encoder", default="libx264", help="libx264 (CPU) u otro encoder de ffmpeg")
    ap.add_argument("--stats-every", type=float, default=5.0, help="segundos entre líneas de estado")
    a = ap.parse_args()

    ctx = zmq.Context.instance()
    sub = ctx.socket(zmq.SUB)
    sub.setsockopt(zmq.RCVHWM, 1)   # igual que image_client: nos quedamos con el último cuadro
    sub.setsockopt(zmq.LINGER, 0)
    sub.connect(a.zmq)
    sub.setsockopt_string(zmq.SUBSCRIBE, "")
    poller = zmq.Poller()
    poller.register(sub, zmq.POLLIN)

    print(f"esperando cuadros de {a.zmq} ...", flush=True)
    if not poller.poll(timeout=10_000):
        print("no llegan cuadros: ¿está corriendo teleimager-server? (~/g1-task/camera.sh status)", file=sys.stderr)
        return 2
    first = sub.recv()
    print(f"primer cuadro: {len(first)} bytes. Lanzando ffmpeg hacia {a.rtsp_url}", flush=True)

    ff = subprocess.Popen(build_ffmpeg_cmd(a.fps, a.bitrate, a.rtsp_url, a.encoder), stdin=subprocess.PIPE)

    stop = False

    def on_signal(*_: object) -> None:
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    frames, dropped, t_stat, t0 = 0, 0, time.monotonic(), time.monotonic()
    try:
        ff.stdin.write(first)
        while not stop:
            if ff.poll() is not None:
                print(f"ffmpeg terminó (código {ff.returncode}): ¿MediaMTX corriendo en el destino?", file=sys.stderr)
                return 3
            if not poller.poll(timeout=1000):
                dropped += 1
                print("1 s sin cuadros de teleimager", file=sys.stderr, flush=True)
                continue
            ff.stdin.write(sub.recv())
            frames += 1
            now = time.monotonic()
            if now - t_stat >= a.stats_every:
                print(f"{frames} cuadros, {frames / (now - t0):.1f} fps promedio, {dropped} segundos sin cuadros", flush=True)
                t_stat = now
    except BrokenPipeError:
        print("ffmpeg cerró la entrada (revisar que MediaMTX esté corriendo)", file=sys.stderr)
        return 3
    finally:
        try:
            ff.stdin.close()
        except Exception:
            pass
        ff.terminate()
        try:
            ff.wait(timeout=5)
        except subprocess.TimeoutExpired:
            ff.kill()
        sub.close()
        print(f"fin: {frames} cuadros en {time.monotonic() - t0:.0f} s", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
