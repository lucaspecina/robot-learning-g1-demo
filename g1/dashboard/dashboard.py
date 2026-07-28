#!/usr/bin/env python3
"""Tablero: ver en vivo todo lo que le pasa al robot.

Se sienta a escuchar todos los canales del sistema y los muestra en una pagina
web: lo que ve la camara, lo que reconoce, donde esta (en un mapa visto desde
arriba), que mission esta ejecutando y en que paso va.

Es la ventana del operador — el equivalente de lo que en un despliegue real
mira el que supervisa al robot. No decide nada ni toca nada: solo observa.

  escucha: /g1/head_cam/image   lo que ve el robot
           /g1/detections       lo que reconoce
           /g1/odom             donde esta y a que velocidad
           /cmd_vel             que se le esta ordenando
           /g1/mobility/status  quien tiene permiso para moverlo
           /g1/goal             a donde lo mandaron
           /g1/nav_status       si la navegacion llego
           /g1/mission_status   el relato de la mission
           /g1/arm_pose         que hacen los arms

Sobre el video: cada cuadro lleva estampado un numero y la hora DENTRO de la
imagen. Si ese text avanza, el video esta vivo — sin discusion posible. El
refresco en el navegador se encadena al onload de la imagen (no a un timer
ciego), asi un error no lo deja congelado en silencio.

Uso (dentro del contenedor jetson):
    python3 dashboard.py
y despues, desde tu maquina, con un tunel al puerto 8080:
    ssh -L 8080:localhost:8080 lucas@<IP>
    y abrir http://localhost:8080
"""
import io
import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String

PORT = 8080
HISTORY_MAX = 60

# La altura del pelvis separa "de pie" de "fallen": parado mide ~0.72 m.
FALLEN_HEIGHT = 0.45

# Sin noticias del robot por mas de estos segundos, lo damos por apagado.
# El robot publica su estado 50 veces por segundo: 3 s de silencio es muchisimo.
OFFLINE_AFTER_S = 3.0

# Estado compartido entre el nodo ROS y el servidor web.
state = {
    "camera_jpeg": None,
    "camera_time": 0.0,
    "odom_time": 0.0,      # cuando llego el ultimo dato del robot
    "frames": 0,
    "detections": {},
    "pose": {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0},
    "real_speed": 0.0,
    "fallen": False,
    "cmd": {"vx": 0.0, "vy": 0.0, "vyaw": 0.0},
    "mobility": {
        "owner": "-",
        "requester": "-",
        "transition_reason": "-",
        "rejected_commands": 0,
    },
    "goal": None,
    "nav": "-",
    "arms": "reposo",
    "mission": [],
}
lock = threading.Lock()


def to_jpeg(img: np.ndarray, text: str) -> bytes:
    """Comprime la imagen y le estampa un text de vida en la franja superior.

    El text (numero de cuadro + hora) viaja DENTRO del JPEG: si en el
    navegador ese numero avanza, el video esta llegando; si esta clavado, el
    problema es del lado del que mira, no del robot.
    """
    from PIL import Image as PILImage, ImageDraw
    im = PILImage.fromarray(img)
    d = ImageDraw.Draw(im)
    d.rectangle([0, 0, im.width, 14], fill=(0, 0, 0))
    d.text((4, 2), text, fill=(0, 255, 140))
    buf = io.BytesIO()
    im.save(buf, format="JPEG", quality=70)
    return buf.getvalue()


class DashboardNode(Node):
    def __init__(self):
        super().__init__("dashboard")
        self.create_subscription(Image, "/g1/head_cam/image", self.on_image, 1)
        self.create_subscription(String, "/g1/detections", self.on_detections, 10)
        self.create_subscription(Odometry, "/g1/odom", self.on_odom, 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        self.create_subscription(
            String,
            "/g1/mobility/status",
            self.on_mobility,
            10,
        )
        self.create_subscription(PoseStamped, "/g1/goal", self.on_goal, 10)
        self.create_subscription(String, "/g1/nav_status", self.on_nav, 10)
        self.create_subscription(String, "/g1/mission_status", self.on_mission, 10)
        self.create_subscription(String, "/g1/arm_pose", self.on_arms, 10)
        self.get_logger().info(f"tablero escuchando; sirve en el puerto {PORT}")

    def on_image(self, msg: Image):
        if msg.encoding != "rgb8":
            return
        # No comprimimos todos los frames: el navegador pide ~4 por segundo.
        if time.time() - state["camera_time"] < 0.15:
            return
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        n = state["frames"] + 1
        text = f"cuadro {n}  {time.strftime('%H:%M:%S')}  (si avanza, es video)"
        try:
            jpeg = to_jpeg(img, text)
        except Exception:
            return
        with lock:
            state["camera_jpeg"] = jpeg
            state["camera_time"] = time.time()
            state["frames"] = n

    def on_detections(self, msg: String):
        try:
            with lock:
                state["detections"] = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def on_odom(self, msg: Odometry):
        p, o = msg.pose.pose.position, msg.pose.pose.orientation
        v = msg.twist.twist.linear
        yaw = math.atan2(2.0 * (o.w * o.z + o.x * o.y),
                         1.0 - 2.0 * (o.y * o.y + o.z * o.z))
        with lock:
            state["pose"] = {"x": round(p.x, 2), "y": round(p.y, 2),
                              "z": round(p.z, 3), "yaw": round(math.degrees(yaw))}
            state["real_speed"] = round(math.hypot(v.x, v.y), 2)
            state["fallen"] = p.z < FALLEN_HEIGHT
            state["odom_time"] = time.time()

    def on_cmd(self, msg: Twist):
        with lock:
            state["cmd"] = {"vx": round(msg.linear.x, 2),
                             "vy": round(msg.linear.y, 2),
                             "vyaw": round(msg.angular.z, 2)}

    def on_mobility(self, msg: String):
        try:
            mobility = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        with lock:
            state["mobility"] = mobility

    def on_goal(self, msg: PoseStamped):
        with lock:
            state["goal"] = {"x": round(msg.pose.position.x, 2),
                              "y": round(msg.pose.position.y, 2)}

    def on_nav(self, msg: String):
        with lock:
            state["nav"] = msg.data
            if msg.data in ("llegue", "cancelado"):
                state["goal"] = None

    def on_arms(self, msg: String):
        with lock:
            state["arms"] = msg.data

    def on_mission(self, msg: String):
        with lock:
            state["mission"].append({"t": time.strftime("%H:%M:%S"), "txt": msg.data})
            del state["mission"][:-HISTORY_MAX]


PAGINA = """<!doctype html>
<html lang="es"><head><meta charset="utf-8"><title>G1 en vivo</title>
<style>
 body{margin:0;background:#12141a;color:#e6e8ee;font:14px/1.5 system-ui,sans-serif}
 header{padding:12px 20px;background:#1a1d26;border-bottom:1px solid #2a2f3d}
 header b{font-size:17px}  header span{color:#8b93a7;margin-left:12px}
 .grid{display:grid;grid-template-columns:1fr 1fr 1fr;gap:16px;padding:16px;max-width:1500px}
 @media(max-width:1100px){.grid{grid-template-columns:1fr 1fr}}
 @media(max-width:760px){.grid{grid-template-columns:1fr}}
 .card{background:#1a1d26;border:1px solid #2a2f3d;border-radius:10px;padding:14px}
 .card h2{margin:0 0 10px;font-size:12px;text-transform:uppercase;
          letter-spacing:.08em;color:#8b93a7;font-weight:600}
 img{width:100%;border-radius:6px;background:#000;display:block}
 canvas{width:100%;border-radius:6px;background:#0d0f14;display:block}
 table{width:100%;border-collapse:collapse}
 td{padding:5px 0;border-bottom:1px solid #232735;vertical-align:top}
 td:last-child{text-align:right;font-variant-numeric:tabular-nums;color:#cfd4e0;
               white-space:nowrap;padding-left:10px}
 small{display:block;color:#5f6675;font-size:11px;line-height:1.35}
 .tag{display:inline-block;padding:2px 9px;border-radius:20px;font-size:12px;
      background:#243044;color:#8fc0ff;margin:2px 4px 2px 0}
 .state-vivo{color:#7ddc9a} .state-fallen{color:#ff8f8f;font-weight:700}
 .log{max-height:260px;overflow-y:auto;font-size:13px}
 .log div{padding:3px 0;border-bottom:1px solid #232735}
 .log .t{color:#5f6675;margin-right:8px}
</style></head><body>
<header><b>G1 — en vivo</b><span id="hb">conectando...</span>
<span id="alerta" class="state-fallen"></span></header>
<div class="grid">

  <div class="card"><h2>Lo que ve el robot</h2>
    <img id="cam" alt="camara">
    <small>La franja negra de arriba trae numero de cuadro y hora, estampados
    por el robot: si avanzan, el video esta vivo. La escena puede estar quieta
    (robot parado mirando algo fijo) — el contador es la prueba de vida.</small>
    <div id="dets" style="margin-top:8px"></div>
    <small>Etiquetas: lo que el detector reconoce por color en este momento.
    "centro" 0.5 = centrado en la imagen; "tamaño" = fraccion del cuadro.</small></div>

  <div class="card"><h2>Mapa (visto desde arriba)</h2>
    <canvas id="mapa" width="360" height="360"></canvas>
    <small>Azul: el robot (la rayita indica hacia donde mira). Estela: por
    donde camino. Cruz verde: objetivo de navegacion vigente. Marron: la mesa.
    Circulo claro: el reloj. El eje x apunta a la derecha, el y hacia arriba.</small></div>

  <div class="card"><h2>Estado del robot</h2>
    <table>
      <tr><td>de pie / fallen<small>altura del pelvis; parado mide ~0.72 m,
        por debajo de 0.45 esta en el piso</small></td><td id="vida">-</td></tr>
      <tr><td>posicion<small>coordenadas (x, y) en metros, en el mapa de la
        habitacion</small></td><td id="pos">-</td></tr>
      <tr><td>rumbo<small>hacia donde mira, en grados; 0 = eje x positivo,
        90 = eje y positivo</small></td><td id="yaw">-</td></tr>
      <tr><td>velocidad real<small>medida del cuerpo, en metros por segundo de
        TIEMPO SIMULADO (el simulador corre al ~20%: se ve 5x mas lento)</small></td>
        <td id="vreal">-</td></tr>
      <tr><td>orden de velocidad<small>lo que la navegacion le pide a las
        piernas: adelante (vx), costado (vy), giro. Es la salida ya
        arbitrada, no la suma de fuentes.</small></td><td id="cmd">-</td></tr>
      <tr><td>dueño de movilidad<small>única fuente autorizada a mover la base;
        los comandos de las demás se descartan</small></td>
        <td id="mobility">-</td></tr>
      <tr><td>navegacion<small>moviendo = yendo a un objetivo; llegue = lo
        alcanzo; guion = sin objetivo</small></td><td id="nav">-</td></tr>
      <tr><td>objetivo<small>a donde lo mando la ultima orden de navegacion,
        si hay una vigente</small></td><td id="goal">-</td></tr>
      <tr><td>arms<small>pose actual: reposo (colgando), listo (extendidos
        adelante), transporte (recogidos contra el cuerpo)</small></td>
        <td id="arm">-</td></tr>
      <tr><td>frames de camara<small>total de imagenes publicadas; si sube,
        los ojos funcionan</small></td><td id="fr">-</td></tr>
    </table></div>

  <div class="card" style="grid-column:1/-1"><h2>La mission, paso a paso</h2>
    <div class="log" id="log"></div>
    <small>El relato del agente: el plan que armo y cada paso que ejecuta.
    Vacio = no hay mission en curso.</small></div>
</div>
<script>
// --- video: el refresco se encadena al onload, no a un timer ciego ---
const cam = document.getElementById('cam');
function refreshCam(){ cam.src = '/camera.jpg?' + Date.now(); }
cam.onload  = () => setTimeout(refreshCam, 300);
cam.onerror = () => { cam.removeAttribute('src'); setTimeout(refreshCam, 1200); };
refreshCam();

// --- mapa ---
const WORLD = {xmin:-1.5, xmax:5.0, ymin:-2.5, ymax:4.0};   // metros visibles
const TABLE  = {x:3.0, y:0.0, width:1.2, depth:0.8};
const CLOCK = {x:0.0, y:2.5};
const trail = [];
function toScreen(c, wx, wy){
  const W = c.width, H = c.height;
  return [ (wx - WORLD.xmin) / (WORLD.xmax - WORLD.xmin) * W,
           H - (wy - WORLD.ymin) / (WORLD.ymax - WORLD.ymin) * H ];
}
function drawMap(s){
  const c = document.getElementById('mapa'), g = c.getContext('2d');
  g.clearRect(0,0,c.width,c.height);
  g.strokeStyle = '#1c2029'; g.lineWidth = 1;
  for(let x = Math.ceil(WORLD.xmin); x <= WORLD.xmax; x++){
    const [px,] = toScreen(c, x, 0);
    g.beginPath(); g.moveTo(px,0); g.lineTo(px,c.height); g.stroke();
  }
  for(let y = Math.ceil(WORLD.ymin); y <= WORLD.ymax; y++){
    const [,py] = toScreen(c, 0, y);
    g.beginPath(); g.moveTo(0,py); g.lineTo(c.width,py); g.stroke();
  }
  // la mesa
  const [mx,my] = toScreen(c, TABLE.x - TABLE.width/2, TABLE.y + TABLE.depth/2);
  const [mx2,my2] = toScreen(c, TABLE.x + TABLE.width/2, TABLE.y - TABLE.depth/2);
  g.fillStyle = '#6b4f2a'; g.fillRect(mx, my, mx2-mx, my2-my);
  g.fillStyle = '#8b93a7'; g.font = '11px sans-serif';
  g.fillText('mesa', mx+4, my+14);
  // el reloj
  const [rx,ry] = toScreen(c, CLOCK.x, CLOCK.y);
  g.beginPath(); g.arc(rx, ry, 8, 0, 7); g.fillStyle = '#d8d8c8'; g.fill();
  g.fillStyle = '#8b93a7'; g.fillText('reloj', rx+11, ry+4);
  // la trail
  if(trail.length > 1){
    g.strokeStyle = '#2f5f8f'; g.lineWidth = 2; g.beginPath();
    trail.forEach((p,i) => { const [px,py] = toScreen(c, p[0], p[1]);
      i ? g.lineTo(px,py) : g.moveTo(px,py); });
    g.stroke();
  }
  // el objetivo
  if(s.goal){
    const [gx,gy] = toScreen(c, s.goal.x, s.goal.y);
    g.strokeStyle = '#7ddc9a'; g.lineWidth = 2;
    g.beginPath(); g.moveTo(gx-7,gy); g.lineTo(gx+7,gy);
    g.moveTo(gx,gy-7); g.lineTo(gx,gy+7); g.stroke();
  }
  // el robot
  const [px,py] = toScreen(c, s.pose.x, s.pose.y);
  g.beginPath(); g.arc(px, py, 7, 0, 7);
  g.fillStyle = s.fallen ? '#ff8f8f' : '#4f8fdc'; g.fill();
  const a = s.pose.yaw * Math.PI / 180;
  g.strokeStyle = '#e6e8ee'; g.lineWidth = 2; g.beginPath();
  g.moveTo(px, py); g.lineTo(px + 13*Math.cos(a), py - 13*Math.sin(a)); g.stroke();
}

// --- state ---
const APAGADO = '—';
let estabaOnline = false;

function apagarPanel(s){
  // Robot apagado: ningun numero viejo en pantalla. Guiones en todo, para que
  // nunca se confunda lo que pasa ahora con lo que paso antes de matarlo.
  ['pos','yaw','vreal','cmd','mobility','nav','goal','arm','vida'].forEach(id =>
    document.getElementById(id).textContent = APAGADO);
  document.getElementById('fr').textContent = APAGADO;
  document.getElementById('dets').innerHTML =
    '<span style="color:#5f6675">sin datos</span>';
  document.getElementById('alerta').textContent =
    '⏻ ROBOT APAGADO' + (s.silencio_s != null
      ? ' — sin datos hace ' + s.silencio_s + ' s' : '');
  document.getElementById('hb').textContent = 'esperando al robot...';
  const c = document.getElementById('mapa'), g = c.getContext('2d');
  g.clearRect(0,0,c.width,c.height);
  g.fillStyle = '#5f6675'; g.font = '13px sans-serif';
  g.fillText('el robot no esta corriendo', 90, 180);
}

async function tick(){
  try{
    const s = await (await fetch('/state')).json();

    if(!s.online){
      trail.length = 0;          // la estela vieja no sobrevive al apagado
      estabaOnline = false;
      apagarPanel(s);
      return setTimeout(tick, 600);
    }
    if(!estabaOnline){           // volvio: arrancamos de cero
      trail.length = 0;
      estabaOnline = true;
    }

    document.getElementById('hb').textContent =
      'actualizado ' + new Date().toLocaleTimeString();
    document.getElementById('alerta').textContent =
      s.fallen ? '⚠ EL ROBOT ESTA EN EL PISO' : '';
    document.getElementById('vida').innerHTML = s.fallen
      ? '<span class="state-fallen">CAIDO (' + s.pose.z + ' m)</span>'
      : '<span class="state-vivo">de pie (' + s.pose.z + ' m)</span>';
    document.getElementById('pos').textContent = `x ${s.pose.x}   y ${s.pose.y}`;
    document.getElementById('yaw').textContent = s.pose.yaw + '°';
    document.getElementById('vreal').textContent = s.real_speed + ' m/s';
    document.getElementById('cmd').textContent =
      `vx ${s.cmd.vx}  vy ${s.cmd.vy}  giro ${s.cmd.vyaw}`;
    document.getElementById('mobility').textContent =
      `${s.mobility.owner}:${s.mobility.requester} · ` +
      `${s.mobility.transition_reason} · descartados ${s.mobility.rejected_commands}`;
    document.getElementById('nav').textContent = s.nav;
    document.getElementById('goal').textContent =
      s.goal ? `(${s.goal.x}, ${s.goal.y})` : '-';
    document.getElementById('arm').textContent = s.arms;
    document.getElementById('fr').textContent =
      s.frames + (s.video_online ? '' : '  (video detenido)');
    document.getElementById('dets').innerHTML = Object.keys(s.detections).length
      ? Object.entries(s.detections).map(([k,v]) =>
          `<span class="tag">${k} · centro ${v.cx} · tamaño ${v.area}</span>`).join('')
      : '<span style="color:#5f6675">no reconoce nada en este momento</span>';
    document.getElementById('log').innerHTML = s.mission.length
      ? s.mission.slice().reverse().map(m =>
          `<div><span class="t">${m.t}</span>${m.txt}</div>`).join('')
      : '';
    const u = trail[trail.length-1];
    if(!u || Math.hypot(u[0]-s.pose.x, u[1]-s.pose.y) > 0.05){
      trail.push([s.pose.x, s.pose.y]);
      if(trail.length > 600) trail.shift();
    }
    drawMap(s);
  }catch(e){
    document.getElementById('hb').textContent = 'sin conexion, reintentando...';
  }
  setTimeout(tick, 400);
}
tick();
</script></body></html>"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _headers(self, code, ctype):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        # Nada de cache: cada pedido trae lo ultimo. Sin esto algunos
        # navegadores clavan la imagen de la camara para siempre.
        self.send_header("Cache-Control", "no-store, must-revalidate")
        self.end_headers()

    def do_GET(self):
        route = self.path.split("?")[0]
        if route == "/":
            self._headers(200, "text/html; charset=utf-8")
            self.wfile.write(PAGINA.encode())
        elif route == "/camera.jpg":
            with lock:
                viejo = time.time() - state["camera_time"] > OFFLINE_AFTER_S
                jpeg = None if viejo else state["camera_jpeg"]
            if jpeg is None:
                self._headers(404, "text/plain")
                self.wfile.write(b"sin imagen todavia")
            else:
                self._headers(200, "image/jpeg")
                self.wfile.write(jpeg)
        elif route == "/state":
            # Un tablero que muestra datos viejos como si fueran de ahora es
            # peor que uno vacio: no se puede distinguir lo que pasa de lo que
            # paso. Si hace mas de OFFLINE_AFTER_S que no llega nada del robot,
            # lo decimos y la pagina apaga todos los valores.
            with lock:
                data = {k: v for k, v in state.items() if k != "camera_jpeg"}
                ahora = time.time()
                data["online"] = (ahora - state["odom_time"]) < OFFLINE_AFTER_S
                data["silencio_s"] = (round(ahora - state["odom_time"])
                                      if state["odom_time"] else None)
                data["video_online"] = (ahora - state["camera_time"]) < OFFLINE_AFTER_S
            self._headers(200, "application/json")
            self.wfile.write(json.dumps(data).encode())
        else:
            self._headers(404, "text/plain")
            self.wfile.write(b"?")


def main():
    rclpy.init()
    node = DashboardNode()

    server = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    print(f"[tablero] sirviendo en http://localhost:{PORT}", flush=True)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
