#!/usr/bin/env python3
"""Escalera de pruebas: de lo mas simple a lo mas complejo.

No tiene sentido probar la mision de 10 pasos si el robot no se sostiene
parado. Cada peldaño verifica UNA cosa, imprime los numeros y da un veredicto
claro. Si un peldaño falla, los de arriba no significan nada.

    python3 checks.py stand    peldaño 0: se queda de pie sin hacer nada?
    python3 checks.py walk     peldaño 1: camina hacia adelante y frena?
    python3 checks.py goto     peldaño 2: llega a un punto y avisa que llego?
    python3 checks.py all      los tres en orden, frenando en el primero que falle

Todo se mide en TIEMPO DE PARED (el del reloj de quien mira). El simulador
corre a ~0.18 de la velocidad real, asi que 30 s de reloj son ~5 s del mundo
del robot: las duraciones estan elegidas para que alcancen igual.

Uso (dentro del contenedor jetson):
    python3 /workspace/g1/tools/checks.py stand
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

STANDING_HEIGHT_MIN = 0.60   # por debajo de esto ya no esta de pie
WALK_SPEED = 0.3             # m/s que le pedimos al caminar
GOTO_DISTANCE = 1.0          # metros hacia adelante para el peldaño 2


class Checker(Node):
    def __init__(self):
        super().__init__("checks")
        self.pose = None
        self.nav = None
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_hold = self.create_publisher(String, "/g1/hold", 10)
        self.pub_goal = self.create_publisher(PoseStamped, "/g1/goal", 10)
        self.create_subscription(Odometry, "/g1/odom", self.on_odom, 10)
        self.create_subscription(String, "/g1/nav_status", self.on_nav, 10)

    def on_odom(self, msg: Odometry):
        p, o = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (o.w * o.z + o.x * o.y),
                         1.0 - 2.0 * (o.y * o.y + o.z * o.z))
        self.pose = (p.x, p.y, p.z, yaw)

    def on_nav(self, msg: String):
        self.nav = msg.data

    def spin_for(self, seconds: float):
        """Atiende ROS durante N segundos de reloj de pared."""
        fin = time.time() + seconds
        while time.time() < fin:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for_pose(self, timeout=20.0):
        fin = time.time() + timeout
        while self.pose is None and time.time() < fin:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.pose is not None

    def hold(self, on: bool):
        """Prende o apaga el modo quieto de la navegacion.

        Antes de manejar al robot a mano hay que pedirle a la navegacion que
        suelte el volante: si los dos publican en /cmd_vel, se pelean y el
        robot no se mueve.
        """
        self.pub_hold.publish(String(data="on" if on else "off"))
        self.spin_for(1.0)

    def send_cmd(self, vx=0.0, vy=0.0, vyaw=0.0):
        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = vx, vy, vyaw
        self.pub_cmd.publish(t)

    def drive(self, seconds: float, vx=0.0, vyaw=0.0):
        """Mantiene un comando durante N segundos (hay que refrescarlo: el
        robot tiene hombre muerto y frena si deja de recibirlo)."""
        fin = time.time() + seconds
        while time.time() < fin:
            self.send_cmd(vx=vx, vyaw=vyaw)
            self.spin_for(0.2)
        self.send_cmd()


def veredicto(ok: bool, detalle: str) -> bool:
    print(f"\n  {'PASA' if ok else 'FALLA'}: {detalle}\n")
    return ok


# --------------------------------------------------------------------------
def check_stand(c: Checker) -> bool:
    """Peldaño 0: parado y quieto, sin ninguna orden, durante 60 s de reloj."""
    print("\n=== PELDAÑO 0: se queda de pie sin hacer nada? ===")
    print("  Nadie le manda ordenes. Solo tiene que sostenerse.")
    print("  Dura 60 s de reloj (~11 s del mundo del robot).\n")

    x0, y0, z0, _ = c.pose
    print(f"  arranca: altura {z0:.3f} m en ({x0:.2f}, {y0:.2f})")

    alturas, cayo_en = [], None
    for i in range(6):
        c.spin_for(10.0)
        x, y, z, _ = c.pose
        alturas.append(z)
        d = math.hypot(x - x0, y - y0)
        estado = "de pie" if z > STANDING_HEIGHT_MIN else "EN EL PISO"
        print(f"  t={10*(i+1):3d}s   altura {z:.3f} m   corrido {d:.2f} m   {estado}")
        if z <= STANDING_HEIGHT_MIN and cayo_en is None:
            cayo_en = 10 * (i + 1)

    x, y, z, _ = c.pose
    deriva = math.hypot(x - x0, y - y0)
    if cayo_en is not None:
        return veredicto(False, f"se cayo a los {cayo_en} s (altura minima "
                                f"{min(alturas):.3f} m). Nada de lo de arriba "
                                f"tiene sentido probar hasta arreglar esto.")
    return veredicto(True, f"aguanto los 60 s de pie (altura {min(alturas):.3f}-"
                           f"{max(alturas):.3f} m). Se corrio {deriva:.2f} m por "
                           f"la deriva conocida de la policy.")


def check_walk(c: Checker) -> bool:
    """Peldaño 1: caminar hacia adelante y frenar, sin navegacion de por medio."""
    print("\n=== PELDAÑO 1: camina hacia adelante y frena? ===")
    print(f"  Le mandamos {WALK_SPEED} m/s directo a las piernas, 40 s de reloj,")
    print("  y despues comando cero. Sin navegacion: esto prueba SOLO caminar.\n")

    c.hold(False)   # la navegacion suelta el volante: manejamos nosotros
    x0, y0, z0, yaw0 = c.pose
    print(f"  arranca: altura {z0:.3f} m en ({x0:.2f}, {y0:.2f}), rumbo {math.degrees(yaw0):.0f} grados")

    c.drive(40.0, vx=WALK_SPEED)
    x1, y1, z1, _ = c.pose
    recorrido = math.hypot(x1 - x0, y1 - y0)
    print(f"  camino:  altura {z1:.3f} m, recorrio {recorrido:.2f} m")

    print("  ahora comando cero: tiene que frenar y quedarse...")
    c.spin_for(20.0)
    x2, y2, z2, yaw2 = c.pose
    despues_de_frenar = math.hypot(x2 - x1, y2 - y1)
    giro = abs(math.degrees(yaw2 - yaw0))
    print(f"  freno:   altura {z2:.3f} m, siguio {despues_de_frenar:.2f} m mas, "
          f"giro {giro:.0f} grados en total")

    c.hold(True)    # devolver el volante a la navegacion
    if z2 <= STANDING_HEIGHT_MIN:
        return veredicto(False, f"termino en el piso (altura {z2:.3f} m)")
    if recorrido < 0.3:
        return veredicto(False, f"casi no avanzo ({recorrido:.2f} m): "
                                f"la orden no esta llegando a las piernas")
    return veredicto(True, f"camino {recorrido:.2f} m de pie y al frenar solo "
                           f"siguio {despues_de_frenar:.2f} m. Desvio de rumbo "
                           f"acumulado: {giro:.0f} grados.")


def check_goto(c: Checker) -> bool:
    """Peldaño 2: la navegacion lleva al robot a un punto y avisa que llego."""
    print("\n=== PELDAÑO 2: llega a un punto y avisa que llego? ===")
    print(f"  Objetivo: {GOTO_DISTANCE} m hacia adelante de donde esta.")
    print("  Ahora si interviene la navegacion. Hasta 3 minutos de reloj.\n")

    x0, y0, z0, yaw0 = c.pose
    gx = x0 + GOTO_DISTANCE * math.cos(yaw0)
    gy = y0 + GOTO_DISTANCE * math.sin(yaw0)
    print(f"  arranca en ({x0:.2f}, {y0:.2f}) -> objetivo ({gx:.2f}, {gy:.2f})")

    c.nav = None
    g = PoseStamped()
    g.pose.position.x, g.pose.position.y = gx, gy
    c.pub_goal.publish(g)

    fin = time.time() + 180.0
    while time.time() < fin:
        c.spin_for(10.0)
        x, y, z, _ = c.pose
        falta = math.hypot(gx - x, gy - y)
        print(f"  t={int(180 - (fin - time.time())):3d}s   "
              f"a {falta:.2f} m del objetivo   altura {z:.3f}   nav: {c.nav}")
        if z <= STANDING_HEIGHT_MIN:
            return veredicto(False, f"se cayo en el camino (altura {z:.3f} m)")
        if c.nav == "llegue":
            return veredicto(True, f"reporto 'llegue' quedando a {falta:.2f} m "
                                   f"del objetivo")

    x, y, _, _ = c.pose
    falta = math.hypot(gx - x, gy - y)
    return veredicto(False, f"nunca reporto 'llegue'. Quedo a {falta:.2f} m del "
                            f"objetivo tras 3 minutos — probablemente orbitando.")


CHECKS = {"stand": check_stand, "walk": check_walk, "goto": check_goto}


def main():
    cual = sys.argv[1] if len(sys.argv) > 1 else "all"
    if cual not in CHECKS and cual != "all":
        print(f"uso: checks.py {{{'|'.join(CHECKS)}|all}}")
        return 1

    rclpy.init()
    c = Checker()
    try:
        if not c.wait_for_pose():
            print("\n  NO LLEGA /g1/odom: el robot no esta corriendo.\n")
            return 1

        secuencia = list(CHECKS) if cual == "all" else [cual]
        for nombre in secuencia:
            if not CHECKS[nombre](c):
                if cual == "all":
                    print(f"  Corto aca: sin '{nombre}' no tiene sentido seguir.\n")
                return 1
        print("  Todos los peldaños pasaron.\n")
        return 0
    finally:
        c.send_cmd()
        c.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
