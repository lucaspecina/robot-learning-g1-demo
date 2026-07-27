#!/usr/bin/env python3
"""Tablero: ver en vivo todo lo que le pasa al robot.

Se sienta a escuchar todos los canales del sistema y los muestra en una pagina
web: lo que ve la camara, lo que reconoce, donde esta, que mision esta
ejecutando y en que paso va.

Es la ventana del operador — el equivalente de lo que en un despliegue real
mira el que supervisa al robot. No decide nada ni toca nada: solo observa.

  escucha: /g1/head_cam/image   lo que ve el robot
           /g1/detections       lo que reconoce
           /g1/odom             donde esta y a que velocidad
           /cmd_vel             que se le esta ordenando
           /g1/nav_status       si la navegacion llego
           /g1/mission_status   el relato de la mision
           /g1/arm_pose         que hacen los brazos

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
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String

PORT = 8080
HISTORIAL_MAX = 60

# Estado compartido entre el nodo ROS y el servidor web.
estado = {
    "camara_jpeg": None,
    "camara_hora": 0.0,
    "detecciones": {},
    "pose": {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0},
    "cmd": {"vx": 0.0, "vy": 0.0, "vyaw": 0.0},
    "nav": "-",
    "brazos": "reposo",
    "mision": [],
    "cuadros": 0,
}
lock = threading.Lock()


def to_jpeg(img: np.ndarray) -> bytes:
    """Comprime la imagen para mandarla al navegador."""
    from PIL import Image as PILImage
    buf = io.BytesIO()
    PILImage.fromarray(img).save(buf, format="JPEG", quality=70)
    return buf.getvalue()


class DashboardNode(Node):
    def __init__(self):
        super().__init__("dashboard")
        self.create_subscription(Image, "/g1/head_cam/image", self.on_image, 1)
        self.create_subscription(String, "/g1/detections", self.on_detections, 10)
        self.create_subscription(Odometry, "/g1/odom", self.on_odom, 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        self.create_subscription(String, "/g1/nav_status", self.on_nav, 10)
        self.create_subscription(String, "/g1/mission_status", self.on_mission, 10)
        self.create_subscription(String, "/g1/arm_pose", self.on_arms, 10)
        self.get_logger().info(f"tablero escuchando; sirve en el puerto {PORT}")

    def on_image(self, msg: Image):
        if msg.encoding != "rgb8":
            return
        # No comprimimos todos los cuadros: el navegador pide ~5 por segundo.
        if time.time() - estado["camara_hora"] < 0.15:
            return
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        try:
            jpeg = to_jpeg(img)
        except Exception:
            return
        with lock:
            estado["camara_jpeg"] = jpeg
            estado["camara_hora"] = time.time()
            estado["cuadros"] += 1

    def on_detections(self, msg: String):
        try:
            with lock:
                estado["detecciones"] = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def on_odom(self, msg: Odometry):
        p, o = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (o.w * o.z + o.x * o.y),
                         1.0 - 2.0 * (o.y * o.y + o.z * o.z))
        with lock:
            estado["pose"] = {"x": round(p.x, 2), "y": round(p.y, 2),
                              "z": round(p.z, 3), "yaw": round(math.degrees(yaw))}

    def on_cmd(self, msg: Twist):
        with lock:
            estado["cmd"] = {"vx": round(msg.linear.x, 2),
                             "vy": round(msg.linear.y, 2),
                             "vyaw": round(msg.angular.z, 2)}

    def on_nav(self, msg: String):
        with lock:
            estado["nav"] = msg.data

    def on_arms(self, msg: String):
        with lock:
            estado["brazos"] = msg.data

    def on_mission(self, msg: String):
        with lock:
            estado["mision"].append({"t": time.strftime("%H:%M:%S"), "txt": msg.data})
            del estado["mision"][:-HISTORIAL_MAX]


PAGINA = """<!doctype html>
<html lang="es"><head><meta charset="utf-8"><title>G1 en vivo</title>
<style>
 body{margin:0;background:#12141a;color:#e6e8ee;font:14px/1.5 system-ui,sans-serif}
 header{padding:12px 20px;background:#1a1d26;border-bottom:1px solid #2a2f3d}
 header b{font-size:17px}  header span{color:#8b93a7;margin-left:12px}
 .grid{display:grid;grid-template-columns:1fr 1fr;gap:16px;padding:16px;max-width:1200px}
 @media(max-width:820px){.grid{grid-template-columns:1fr}}
 .card{background:#1a1d26;border:1px solid #2a2f3d;border-radius:10px;padding:14px}
 .card h2{margin:0 0 10px;font-size:12px;text-transform:uppercase;
          letter-spacing:.08em;color:#8b93a7;font-weight:600}
 img{width:100%;border-radius:6px;background:#000;display:block}
 table{width:100%;border-collapse:collapse}
 td{padding:4px 0;border-bottom:1px solid #232735}
 td:last-child{text-align:right;font-variant-numeric:tabular-nums;color:#cfd4e0}
 .tag{display:inline-block;padding:2px 9px;border-radius:20px;font-size:12px;
      background:#243044;color:#8fc0ff;margin:2px 4px 2px 0}
 .log{max-height:300px;overflow-y:auto;font-size:13px}
 .log div{padding:3px 0;border-bottom:1px solid #232735}
 .log .t{color:#5f6675;margin-right:8px}
 .ok{color:#7ddc9a} .no{color:#ff8f8f}
</style></head><body>
<header><b>G1 — en vivo</b><span id="hb">conectando...</span></header>
<div class="grid">
  <div class="card"><h2>Lo que ve el robot</h2>
    <img id="cam" src="/camera.jpg" alt="camara">
    <div id="dets" style="margin-top:10px"></div></div>
  <div class="card"><h2>Estado</h2>
    <table>
      <tr><td>posición</td><td id="pos">-</td></tr>
      <tr><td>altura</td><td id="alt">-</td></tr>
      <tr><td>rumbo</td><td id="yaw">-</td></tr>
      <tr><td>orden de velocidad</td><td id="cmd">-</td></tr>
      <tr><td>navegación</td><td id="nav">-</td></tr>
      <tr><td>brazos</td><td id="arm">-</td></tr>
      <tr><td>cuadros de cámara</td><td id="fr">-</td></tr>
    </table></div>
  <div class="card" style="grid-column:1/-1"><h2>La misión, paso a paso</h2>
    <div class="log" id="log"></div></div>
</div>
<script>
setInterval(()=>{document.getElementById('cam').src='/camera.jpg?'+Date.now()},250);
async function tick(){
  try{
    const s=await (await fetch('/state')).json();
    document.getElementById('hb').textContent='actualizado '+new Date().toLocaleTimeString();
    document.getElementById('pos').textContent=`x ${s.pose.x}  y ${s.pose.y}`;
    document.getElementById('alt').textContent=s.pose.z+' m';
    document.getElementById('yaw').textContent=s.pose.yaw+'°';
    document.getElementById('cmd').textContent=
      `vx ${s.cmd.vx}  vy ${s.cmd.vy}  giro ${s.cmd.vyaw}`;
    document.getElementById('nav').textContent=s.nav;
    document.getElementById('arm').textContent=s.brazos;
    document.getElementById('fr').textContent=s.cuadros;
    document.getElementById('dets').innerHTML=Object.keys(s.detecciones).length
      ? Object.entries(s.detecciones).map(([k,v])=>
          `<span class="tag">${k} · centro ${v.cx} · tamaño ${v.area}</span>`).join('')
      : '<span style="color:#5f6675">no reconoce nada ahora mismo</span>';
    document.getElementById('log').innerHTML=s.mision.slice().reverse().map(m=>{
      const c=/complet|llegue|encontre|veo|OK/i.test(m.txt)?'ok'
             :/FALLO|no /i.test(m.txt)?'no':'';
      return `<div><span class="t">${m.t}</span><span class="${c}">${m.txt}</span></div>`;
    }).join('');
  }catch(e){document.getElementById('hb').textContent='sin conexión con el tablero';}
}
setInterval(tick,400); tick();
</script></body></html>"""


class Handler(BaseHTTPRequestHandler):
    def _send(self, code, tipo, cuerpo):
        self.send_response(code)
        self.send_header("Content-Type", tipo)
        self.send_header("Content-Length", str(len(cuerpo)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(cuerpo)

    def do_GET(self):
        if self.path.startswith("/camera.jpg"):
            with lock:
                jpeg = estado["camara_jpeg"]
            if jpeg is None:
                self._send(404, "text/plain", b"todavia no hay imagen")
            else:
                self._send(200, "image/jpeg", jpeg)
        elif self.path.startswith("/state"):
            with lock:
                cuerpo = json.dumps({k: v for k, v in estado.items()
                                     if k != "camara_jpeg"}).encode()
            self._send(200, "application/json", cuerpo)
        else:
            self._send(200, "text/html; charset=utf-8", PAGINA.encode("utf-8"))

    def log_message(self, *args):
        pass


def main():
    rclpy.init()
    node = DashboardNode()
    servidor = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    threading.Thread(target=servidor.serve_forever, daemon=True).start()
    print(f"tablero sirviendo en el puerto {PORT}", flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    servidor.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
