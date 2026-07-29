#!/usr/bin/env python3
"""El agente: recibe una mision en lenguaje natural y la ejecuta con skills.

Es la cima del sistema. No toca motores ni velocidades: solo decide QUE hacer
y en que orden, llamando a las capacidades que el robot ya tiene. Cada skill se
encarga de su parte y le reporta si salio bien.

  recibe:  /g1/mission         (std_msgs/String — la mision, en castellano)
           /g1/detections      (lo que la camara reconoce)
           /g1/clock_crop/compressed (recorte del reloj)
           /g1/nav_status      (si la navegacion llego)
  publica: /g1/goal            (a donde ir)
           /g1/arm_pose        (que hacer con los brazos)
           /g1/mission_status  (como viene la mision)

Corre en la Jetson como ejecutor local de la misión. Las decisiones que
necesitan modelos grandes se piden por HTTP al servidor externo. Si ese enlace
falla, aborta el paso y el control local deja al robot estable.

Sobre el planificador: hoy resuelve la mision con reglas. La estructura ya esta
lista para que un modelo de lenguaje arme el plan (ver `plan_con_llm`): el
catalogo de skills y el formato del plan son los que se le pasarian al modelo.
Cuando haya credenciales, se cambia el planificador y el resto sigue igual.
"""
import json
import math
import os
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from intelligence_client import (
    IntelligenceClient,
    RemoteIntelligenceError,
)

# El mapa semantico: donde PARARSE para usar cada cosa.
#
# Ojo con esto, que es un error facil de cometer: los destinos de navegacion no
# son las coordenadas del objeto sino POSES DE APROXIMACION — el lugar desde el
# cual el robot puede usarlo. Si el destino fuera el centro de la mesa, el robot
# caminaria contra el mueble, quedaria trabado a medio metro y la navegacion
# nunca daria por cumplido el objetivo (nos paso).
SEMANTIC_MAP = {
    # Cada entrada es (x, y, yaw): no alcanza con llegar al lugar; la postura
    # final debe dejar el sensor mirando al elemento que se quiere usar.
    "mesa": (1.8, 0.0, 0.0),
    "reloj": (0.8, 1.8, 2.4228),
}

# El catalogo de skills: lo que el robot sabe hacer. Esto es, literalmente, lo
# que se le describiria a un modelo de lenguaje para que arme un plan.
SKILLS = {
    "ir_a": "ir_a(lugar) — camina hasta un lugar del mapa: mesa, reloj",
    "mirar": "mirar(objeto) — informa si lo ve y donde: reloj, botella, persona_roja, persona_azul",
    "brazos": "brazos(pose) — mueve los brazos: reposo, listo, transporte",
    "buscar_persona": "buscar_persona(color) — gira hasta encontrar a la persona de ese color",
    "decir": "decir(texto) — le habla a la gente",
}


class Agent(Node):
    def __init__(self):
        super().__init__("agent")
        self.detections = {}
        self.nav_status = None
        self.mission_thread = None
        self.clock_crop = None
        self.clock_crop_received_at = None
        self.intelligence = IntelligenceClient()

        self.pub_goal = self.create_publisher(PoseStamped, "/g1/goal", 10)
        self.pub_arms = self.create_publisher(String, "/g1/arm_pose", 10)
        self.pub_status = self.create_publisher(String, "/g1/mission_status", 10)
        self.create_subscription(String, "/g1/mission", self.on_mission, 10)
        self.create_subscription(String, "/g1/detections", self.on_detections, 10)
        self.create_subscription(
            CompressedImage,
            "/g1/clock_crop/compressed",
            self.on_clock_crop,
            2,
        )
        self.create_subscription(String, "/g1/nav_status", self.on_nav_status, 10)

        self.get_logger().info("agente listo. Esperando misiones en /g1/mission")

    # ---------- entradas ----------

    def on_detections(self, msg: String):
        try:
            self.detections = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def on_nav_status(self, msg: String):
        self.nav_status = msg.data

    def on_clock_crop(self, msg: CompressedImage):
        self.clock_crop = bytes(msg.data)
        self.clock_crop_received_at = time.monotonic()

    def on_mission(self, msg: String):
        """Arranca la mision en un hilo aparte.

        No se puede ejecutar aca mismo: este metodo corre DENTRO del sistema de
        mensajes de ROS, y quedarse esperando adentro lo bloquea (el error es
        explicito: "Executor is already spinning"). El hilo lee las variables
        que los callbacks van actualizando.
        """
        if self.mission_thread and self.mission_thread.is_alive():
            self.reportar("ya hay una mision en curso; ignoro la nueva")
            return
        self.mission_thread = threading.Thread(target=self._run_mission,
                                               args=(msg.data,), daemon=True)
        self.mission_thread.start()

    def _run_mission(self, mision: str):
        self.get_logger().info(f"mision recibida: {mision}")
        self.reportar(f"planificando: {mision}")
        plan = self.planificar(mision)
        self.reportar("plan: " + " -> ".join(p[0] + "(" + str(p[1]) + ")" for p in plan))
        self.ejecutar(plan)

    # ---------- planificacion ----------

    def planificar(self, mision: str):
        """Devuelve una lista de pasos [(skill, argumento), ...]."""
        if os.environ.get("AGENT_LLM_ENDPOINT"):
            return self.plan_con_llm(mision)
        return self.plan_con_reglas(mision)

    def plan_con_reglas(self, mision: str):
        """Planificador simple: reconoce la mision de la demo.

        Deliberadamente pobre: existe para que el sistema funcione de punta a
        punta sin depender de un servicio externo. El plan que arma es el mismo
        que armaria el modelo de lenguaje.
        """
        m = mision.lower()
        if "reloj" in m or "hora" in m:
            return [
                ("ir_a", "reloj"),
                ("mirar", "reloj"),
                ("decidir_color", None),      # la rama: segun la hora
                ("ir_a", "mesa"),
                ("brazos", "listo"),
                ("mirar", "botella"),
                ("agarrar", None),            # pendiente: lo hara el VLA
                ("brazos", "transporte"),
                ("buscar_persona", "<color>"),  # se completa al decidir
                ("decir", "aca tenes"),
            ]
        if "mesa" in m:
            return [("ir_a", "mesa"), ("brazos", "listo")]
        return [("decir", "no entendi la mision")]

    def plan_con_llm(self, mision: str):
        """Arma el plan con un modelo de lenguaje.

        Sin implementar todavia: falta decidir el proveedor y cargar
        credenciales. El prompt seria la mision + el catalogo SKILLS + el mapa
        semantico, y la respuesta esperada, la misma lista de pasos que produce
        el planificador de reglas.
        """
        self.get_logger().warn("planificador LLM no configurado; uso reglas")
        return self.plan_con_reglas(mision)

    # ---------- ejecucion ----------

    def ejecutar(self, plan):
        color_objetivo = None

        for skill, arg in plan:
            if skill == "decidir_color":
                hora = self.leer_reloj()
                if hora is None:
                    self.reportar(
                        "FALLO al leer el reloj — misión abortada"
                    )
                    return
                color_objetivo = "persona_roja" if hora < 6 else "persona_azul"
                self.reportar(f"el reloj marca {hora}:00 -> {color_objetivo}")
                continue

            if arg == "<color>":
                arg = color_objetivo or "persona_roja"

            self.reportar(f"ejecutando {skill}({arg})")
            ok = self.ejecutar_skill(skill, arg)
            if not ok:
                self.reportar(f"FALLO en {skill}({arg}) — mision abortada")
                return

        self.reportar("mision completada")

    def ejecutar_skill(self, skill: str, arg) -> bool:
        if skill == "ir_a":
            if arg not in SEMANTIC_MAP:
                return False
            x, y, yaw = SEMANTIC_MAP[arg]
            goal = PoseStamped()
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.header.frame_id = "odom"
            goal.pose.position.x, goal.pose.position.y = float(x), float(y)
            goal.pose.orientation.z = math.sin(yaw / 2.0)
            goal.pose.orientation.w = math.cos(yaw / 2.0)
            self.nav_status = None
            self.pub_goal.publish(goal)
            return self.esperar_llegada()

        if skill == "brazos":
            self.pub_arms.publish(String(data=str(arg)))
            self.dormir(3.0)
            return True

        if skill == "mirar":
            visto = self.esperar_deteccion(str(arg), timeout_s=5.0)
            if visto:
                self.reportar(f"veo {arg}: {visto}")
            else:
                self.reportar(f"no veo {arg}")
            return True   # no ver algo no aborta la mision; el plan decide

        if skill == "buscar_persona":
            return self.buscar(str(arg))

        if skill == "decir":
            self.reportar(f'dice: "{arg}"')
            return True

        if skill == "agarrar":
            self.reportar("agarrar: pendiente (lo va a hacer el VLA)")
            return True

        self.reportar(f"skill desconocida: {skill}")
        return False

    # ---------- utilidades ----------

    def leer_reloj(self):
        """La hora que marca el reloj.

        El detector local publica un recorte pequeño. El modelo vive detrás del
        servidor externo para que una demora o caída de Azure no afecte el
        equilibrio ni el control de movimiento.
        """
        if (
            self.clock_crop is None
            or self.clock_crop_received_at is None
            or time.monotonic() - self.clock_crop_received_at > 10.0
        ):
            self.reportar("no hay una imagen reciente del reloj")
            return None
        try:
            reading = self.intelligence.read_clock(self.clock_crop)
        except RemoteIntelligenceError as error:
            self.reportar(f"servidor de visión no disponible: {error}")
            return None
        if not reading["readable"]:
            self.reportar("el modelo informa que el reloj no es legible")
            return None
        self.reportar(
            f"lectura visual: {reading['text']} "
            f"({reading.get('elapsed_s', '?')} s en el servidor)"
        )
        return int(reading["hour"])

    def esperar_llegada(self, timeout_s: float = None) -> bool:
        """Espera a que la navegacion reporte que llego.

        El tiempo limite se lee del entorno porque depende de cuan rapido corra
        el simulador: con el simulador al 20 % de la velocidad real, recorrer
        8 metros lleva mas de dos minutos de reloj de pared. La solucion de
        fondo es usar el reloj simulado de ROS 2 (/clock + use_sim_time) para
        que los plazos se midan en tiempo de simulacion.
        """
        if timeout_s is None:
            timeout_s = float(os.environ.get("NAV_TIMEOUT_S", "600"))
        inicio = time.time()
        while time.time() - inicio < timeout_s:
            time.sleep(0.2)
            if self.nav_status == "llegue":
                return True
        self.reportar("la navegacion no llego a tiempo")
        return False

    def esperar_deteccion(self, objeto: str, timeout_s: float = 5.0):
        inicio = time.time()
        while time.time() - inicio < timeout_s:
            time.sleep(0.2)
            if objeto in self.detections:
                return self.detections[objeto]
        return None

    def buscar(self, objeto: str, timeout_s: float = 60.0) -> bool:
        """Comportamiento de busqueda: girar hasta tenerlo al frente.

        Gira publicando objetivos de navegacion relativos seria mas prolijo;
        por ahora usamos la deteccion directa: si aparece en la imagen y esta
        razonablemente centrado, damos por encontrado.
        """
        inicio = time.time()
        while time.time() - inicio < timeout_s:
            time.sleep(0.2)
            visto = self.detections.get(objeto)
            if visto and abs(visto["cx"] - 0.5) < 0.25:
                self.reportar(f"encontre {objeto} (centro {visto['cx']}, tamaño {visto['area']})")
                return True
        self.reportar(f"no encontre {objeto}")
        return False

    def dormir(self, segundos: float):
        time.sleep(segundos)

    def reportar(self, texto: str):
        self.get_logger().info(texto)
        self.pub_status.publish(String(data=texto))


def main():
    rclpy.init()
    node = Agent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
