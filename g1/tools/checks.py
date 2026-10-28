#!/usr/bin/env python3
"""Escalera de pruebas: de lo mas simple a lo mas complejo.

No tiene sentido probar la mision de 10 pasos si el robot no se sostiene
parado. Cada peldaño verifica UNA cosa, imprime los numeros y da un veredicto
claro. Si un peldaño falla, los de arriba no significan nada.

    python3 checks.py authority peldaño 0: hay un solo dueño y cancela bien?
    python3 checks.py stand     peldaño 1: se queda de pie sin hacer nada?
    python3 checks.py walk      peldaño 2: camina hacia adelante y frena?
    python3 checks.py turn      peldaño 3: el signo del giro es correcto?
    python3 checks.py goto      peldaño 4: llega a un punto y avisa que llego?
    python3 checks.py all      los tres en orden, frenando en el primero que falle

Todo se mide en TIEMPO DE PARED (el del reloj de quien mira). El simulador
corre a ~0.18 de la velocidad real, asi que 30 s de reloj son ~5 s del mundo
del robot: las duraciones estan elegidas para que alcancen igual.

Uso (dentro del contenedor jetson):
    python3 /workspace/g1/tools/checks.py stand
"""
import json
import math
import sys
import time
import uuid
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

from motion_quality import arm_tracking_metrics, motion_quality_metrics  # noqa: E402

STANDING_HEIGHT_MIN = 0.60   # por debajo de esto ya no esta de pie
STAND_MAX_ERROR_M = 0.15     # sobre de espera libre; manipular exige mucho menos
WALK_SPEED = 0.3             # m/s que le pedimos al caminar
GOTO_DISTANCE = 1.0          # metros hacia adelante para el peldaño 2

# Avanzar no alcanza: una trayectoria oblicua deja de ser útil al acercarse a
# una mesa. El ángulo hace comparable la prueba aunque cambie el RTF y, por lo
# tanto, la distancia recorrida durante los mismos segundos de reloj.
# La referencia oficial de Isaac midió entre 2,6° y 19,7° sin cambiar nada.
# Esta prueba sólo detecta errores gruesos de locomoción; seguir una línea y
# llegar con precisión son responsabilidades de navegación.
WALK_MAX_PATH_ANGLE_DEG = 10.0
WALK_MAX_YAW_ERROR_DEG = 10.0
WALK_MAX_BRAKE_M = 0.20
TURN_SPEED = 0.3
TURN_DURATION_S = 15.0
TURN_MIN_RESPONSE_DEG = 5.0


class Checker(Node):
    def __init__(self, payload_kg: float = None):
        super().__init__("checks")
        self.pose = None
        self.nav = None
        self.mobility_owner = None
        self.robot_mode = None
        self.arm_status = None
        self.payload_status = None
        self.test_payload_kg = payload_kg
        self.motion_capture_phase = None
        self.motion_samples = {}
        self.arm_motion_samples = {}
        self.pub_cmd = self.create_publisher(Twist, "/g1/cmd_vel/test", 10)
        self.pub_mobility = self.create_publisher(
            String, "/g1/mobility/request", 10
        )
        self.pub_goal = self.create_publisher(PoseStamped, "/g1/goal", 10)
        self.pub_reset = self.create_publisher(String, "/g1/reset", 10)
        self.pub_arm_pose = self.create_publisher(String, "/g1/arm_pose", 10)
        self.pub_payload = self.create_publisher(
            String, "/g1/payload_request", 10
        )
        self.create_subscription(Odometry, "/g1/odom", self.on_odom, 10)
        self.create_subscription(String, "/g1/nav_status", self.on_nav, 10)
        self.create_subscription(
            String,
            "/g1/mobility/status",
            self.on_mobility_status,
            10,
        )
        self.create_subscription(
            String,
            "/g1/robot_status",
            self.on_robot_status,
            10,
        )
        self.create_subscription(String, "/g1/arm_status", self.on_arm_status, 10)
        self.create_subscription(
            String,
            "/g1/payload_status",
            self.on_payload_status,
            10,
        )

    def on_odom(self, msg: Odometry):
        p, o = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (o.w * o.z + o.x * o.y),
                         1.0 - 2.0 * (o.y * o.y + o.z * o.z))
        self.pose = (p.x, p.y, p.z, yaw)
        if self.motion_capture_phase is not None:
            roll = math.atan2(
                2.0 * (o.w * o.x + o.y * o.z),
                1.0 - 2.0 * (o.x * o.x + o.y * o.y),
            )
            pitch_term = max(
                -1.0,
                min(1.0, 2.0 * (o.w * o.y - o.z * o.x)),
            )
            self.motion_samples.setdefault(
                self.motion_capture_phase,
                [],
            ).append({
                "roll_rad": roll,
                "pitch_rad": math.asin(pitch_term),
                "height_m": p.z,
                "angular_x_radps": msg.twist.twist.angular.x,
                "angular_y_radps": msg.twist.twist.angular.y,
            })

    def on_nav(self, msg: String):
        self.nav = msg.data

    def on_mobility_status(self, msg: String):
        try:
            self.mobility_owner = json.loads(msg.data)["owner"]
        except (json.JSONDecodeError, KeyError, TypeError):
            pass

    def on_robot_status(self, msg: String):
        try:
            self.robot_mode = json.loads(msg.data)["mode"]
        except (json.JSONDecodeError, KeyError, TypeError):
            pass

    def on_arm_status(self, msg: String):
        try:
            self.arm_status = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if (
            self.motion_capture_phase is None
            or self.arm_status.get("controller") != "pink"
        ):
            return
        try:
            sample = {
                "position_error_m": float(
                    self.arm_status["maximum_wrist_position_error_m"]
                ),
                "orientation_error_deg": float(
                    self.arm_status["maximum_wrist_orientation_error_deg"]
                ),
                "joint_error_rad": float(self.arm_status["max_error_rad"]),
                "reached": bool(self.arm_status["reached"]),
            }
        except (KeyError, TypeError, ValueError):
            return
        self.arm_motion_samples.setdefault(
            self.motion_capture_phase,
            [],
        ).append(sample)

    def on_payload_status(self, msg: String):
        try:
            self.payload_status = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

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

    def request_mobility(
        self,
        operation: str,
        reason: str = None,
        source: str = "test",
        requester: str = "checks",
    ):
        request = {
            "operation": operation,
            "source": source,
            "requester": requester,
        }
        if reason is not None:
            request["reason"] = reason
        self.pub_mobility.publish(String(data=json.dumps(request)))

    def acquire_test_mobility(self, timeout_s: float = 5.0) -> bool:
        """Espera confirmación: publicar al topic de prueba no alcanza."""
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            self.request_mobility("acquire")
            self.spin_for(0.25)
            if self.mobility_owner == "test":
                return True
        return False

    def release_test_mobility(self, reason: str):
        self.send_cmd()
        self.request_mobility("release", reason)
        self.spin_for(0.25)

    def prepare_payload(self, timeout_s: float = 40.0) -> bool:
        """Restaura pose y masa porque reset las elimina deliberadamente."""
        if self.test_payload_kg is None:
            return True
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self.pub_arm_pose.publish(String(data="transporte"))
            self.spin_for(0.2)
            status = self.arm_status or {}
            if status.get("pose") == "transporte" and status.get("reached"):
                break
        else:
            print("  FALLA: los brazos no confirmaron la pose transporte")
            return False

        request_id = str(uuid.uuid4())
        command = "attach" if self.test_payload_kg > 0.0 else "detach"
        request = String(data=json.dumps({
            "request_id": request_id,
            "command": command,
            "mass_kg": self.test_payload_kg,
        }))
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            self.pub_payload.publish(request)
            self.spin_for(0.25)
            status = self.payload_status or {}
            if status.get("request_id") != request_id:
                continue
            applied = float(status.get("applied_mass_kg", -1.0))
            expected_state = "attached" if self.test_payload_kg > 0.0 else "detached"
            valid = status.get("state") == expected_state and abs(
                applied - self.test_payload_kg
            ) <= 1e-3
            if valid:
                print(
                    f"  carga confirmada: {applied:.2f} kg en "
                    f"{status.get('attachment_points', [])}"
                )
                return True
            print(f"  FALLA: carga rechazada: {status.get('error', status)}")
            return False
        print("  FALLA: el robot no confirmó la carga después del reinicio")
        return False

    def reset_robot(self):
        """Devuelve el banco al origen para no encadenar posiciones peligrosas."""
        self.pub_reset.publish(String(data="prueba reproducible"))
        # El reinicio se aplica dentro del lazo de física. Esperar también deja
        # que navegación detecte el salto y entregue cualquier concesión vieja.
        self.spin_for(2.0)
        if not self.prepare_payload():
            raise RuntimeError("no se restauró la carga experimental")

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

    def start_motion_capture(self, phase: str):
        self.motion_samples = {phase: []}
        self.arm_motion_samples = {phase: []}
        self.motion_capture_phase = phase

    def change_motion_capture_phase(self, phase: str):
        self.motion_samples.setdefault(phase, [])
        self.arm_motion_samples.setdefault(phase, [])
        self.motion_capture_phase = phase

    def stop_motion_capture(self):
        self.motion_capture_phase = None


def print_motion_quality(label: str, metrics: dict):
    print(
        f"  {label}: balance lateral {metrics['roll_p90_span_deg']:.1f}°, "
        f"frontal {metrics['pitch_p90_span_deg']:.1f}°, "
        f"inclinación p95 {metrics['tilt_p95_deg']:.1f}°"
    )
    print(
        f"            oscilación angular RMS "
        f"{metrics['angular_speed_rms_radps']:.2f} rad/s, "
        f"rebote vertical {metrics['height_p90_span_m'] * 100:.1f} cm "
        f"({metrics['sample_count']} muestras)"
    )


def print_arm_tracking(label: str, samples: list[dict]):
    if len(samples) < 2:
        print(f"  {label}: sin suficientes muestras de Pink")
        return
    metrics = arm_tracking_metrics(samples)
    print(
        f"  {label}: error de muñeca p95 "
        f"{metrics['position_error_p95_m'] * 100:.1f} cm / "
        f"{metrics['orientation_error_p95_deg']:.1f}°, máximo "
        f"{metrics['position_error_max_m'] * 100:.1f} cm / "
        f"{metrics['orientation_error_max_deg']:.1f}°"
    )
    print(
        f"            error articular p95 "
        f"{math.degrees(metrics['joint_error_p95_rad']):.1f}°, "
        f"dentro de tolerancia {metrics['reached_fraction'] * 100:.0f}% "
        f"({metrics['sample_count']} muestras)"
    )


def veredicto(ok: bool, detalle: str) -> bool:
    print(f"\n  {'PASA' if ok else 'FALLA'}: {detalle}\n")
    return ok


# --------------------------------------------------------------------------
def check_authority(c: Checker) -> bool:
    """Peldaño 0: una intervención manual cancela una navegación vieja."""
    print("\n=== PELDAÑO 0: la movilidad tiene un solo dueño? ===")
    print("  Inicio una navegación, la interrumpo como operador y verifico")
    print("  que no vuelva a arrancar sola cuando el operador libera el control.\n")

    c.reset_robot()
    x0, y0, _, yaw0 = c.pose
    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.pose.position.x = x0 + 1.0 * math.cos(yaw0)
    goal.pose.position.y = y0 + 1.0 * math.sin(yaw0)
    c.nav = None
    c.pub_goal.publish(goal)

    end = time.monotonic() + 5.0
    while c.mobility_owner != "navigation" and time.monotonic() < end:
        c.spin_for(0.1)
    if c.mobility_owner != "navigation":
        return veredicto(
            False,
            f"navegación no obtuvo la movilidad; dueño: {c.mobility_owner}",
        )

    c.request_mobility(
        "acquire",
        source="manual",
        requester="authority_check",
    )
    end = time.monotonic() + 0.5
    while (
        (c.mobility_owner != "manual" or c.nav != "cancelado")
        and time.monotonic() < end
    ):
        c.spin_for(0.05)

    preempted = c.mobility_owner == "manual" and c.nav == "cancelado"
    c.request_mobility(
        "release",
        "prueba de intervención manual terminada",
        source="manual",
        requester="authority_check",
    )
    c.spin_for(1.5)

    resumed = c.mobility_owner == "navigation"
    c.reset_robot()
    if not preempted:
        return veredicto(
            False,
            f"la intervención no canceló limpiamente: dueño "
            f"{c.mobility_owner}, estado de navegación {c.nav}",
        )
    if resumed:
        return veredicto(
            False,
            "la navegación vieja recuperó control sin recibir un objetivo nuevo",
        )
    return veredicto(
        True,
        f"manual interrumpió, navegación confirmó 'cancelado' y el dueño "
        f"quedó en {c.mobility_owner}",
    )


# --------------------------------------------------------------------------
def check_stand(c: Checker) -> bool:
    """Peldaño 0: equilibrio y mantenimiento de pose durante 60 s de reloj."""
    print("\n=== PELDAÑO 0: se mantiene de pie y cerca de su anclaje? ===")
    print("  stand_hold es el único dueño; navegación y pruebas no intervienen.")
    print("  Dura 60 s de reloj (~11 s del mundo del robot).\n")

    # Cada peldaño limpia su propio estado para que repetirlo no dependa de la
    # carga o la postura que dejó la corrida anterior.
    c.reset_robot()
    c.spin_for(0.5)
    if c.mobility_owner != "stand":
        return veredicto(
            False,
            f"la movilidad pertenece a {c.mobility_owner}; esta prueba necesita "
            "stand_hold como único dueño",
        )

    x0, y0, z0, _ = c.pose
    print(f"  arranca: altura {z0:.3f} m en ({x0:.2f}, {y0:.2f})")

    alturas, errores, cayo_en = [], [], None
    for i in range(60):
        c.spin_for(1.0)
        x, y, z, _ = c.pose
        alturas.append(z)
        d = math.hypot(x - x0, y - y0)
        errores.append(d)
        estado = "de pie" if z > STANDING_HEIGHT_MIN else "EN EL PISO"
        if (i + 1) % 10 == 0:
            print(f"  t={i+1:3d}s   altura {z:.3f} m   error {d:.2f} m   {estado}")
        if z <= STANDING_HEIGHT_MIN and cayo_en is None:
            cayo_en = i + 1
        if c.mobility_owner != "stand":
            return veredicto(
                False,
                "otra operación tomó la movilidad durante quietud; "
                f"dueño observado: {c.mobility_owner}",
            )

    x, y, z, _ = c.pose
    deriva = math.hypot(x - x0, y - y0)
    p95 = sorted(errores)[round(0.95 * (len(errores) - 1))]
    max_error = max(errores)
    print(f"  error de posición: final {deriva:.2f} m, "
          f"percentil 95 {p95:.2f} m, máximo {max_error:.2f} m")
    if cayo_en is not None:
        return veredicto(False, f"se cayo a los {cayo_en} s (altura minima "
                                f"{min(alturas):.3f} m). Nada de lo de arriba "
                                f"tiene sentido probar hasta arreglar esto.")
    if max_error > STAND_MAX_ERROR_M:
        return veredicto(
            False,
            f"se mantuvo de pie pero salió del sobre de {STAND_MAX_ERROR_M:.2f} m "
            f"(máximo {max_error:.2f} m)",
        )
    return veredicto(True, f"aguanto los 60 s de pie (altura {min(alturas):.3f}-"
                           f"{max(alturas):.3f} m). El error final respecto del "
                           f"anclaje fue {deriva:.2f} m y el percentil 95 "
                           f"{p95:.2f} m.")


def check_walk(c: Checker) -> bool:
    """Peldaño 1: caminar hacia adelante y frenar, sin navegacion de por medio."""
    print("\n=== PELDAÑO 1: camina hacia adelante y frena? ===")
    print(f"  Le mandamos {WALK_SPEED} m/s directo a las piernas, 40 s de reloj,")
    print("  y despues comando cero. Sin navegacion: esto prueba SOLO caminar.\n")

    c.reset_robot()
    if not c.acquire_test_mobility():
        return veredicto(
            False,
            f"el árbitro no concedió TEST; dueño actual: {c.mobility_owner}",
        )
    x0, y0, z0, yaw0 = c.pose
    print(f"  arranca: altura {z0:.3f} m en ({x0:.2f}, {y0:.2f}), rumbo {math.degrees(yaw0):.0f} grados")

    c.start_motion_capture("walking")
    try:
        c.drive(40.0, vx=WALK_SPEED)
        walking_quality = motion_quality_metrics(
            c.motion_samples["walking"]
        )
        x1, y1, z1, yaw1 = c.pose
        dx, dy = x1 - x0, y1 - y0
        forward = dx * math.cos(yaw0) + dy * math.sin(yaw0)
        lateral = -dx * math.sin(yaw0) + dy * math.cos(yaw0)
        recorrido = math.hypot(x1 - x0, y1 - y0)
        path_angle = abs(math.degrees(math.atan2(lateral, max(forward, 1e-9))))
        walk_yaw_error = abs(math.degrees(math.atan2(
            math.sin(yaw1 - yaw0), math.cos(yaw1 - yaw0)
        )))
        print(
            f"  camino:  altura {z1:.3f} m, adelante {forward:.2f} m, "
            f"costado {lateral:+.2f} m"
        )
        print(
            f"            trayectoria {path_angle:.1f} grados fuera de la recta, "
            f"cuerpo girado {walk_yaw_error:.1f} grados"
        )
        print_motion_quality("torso caminando", walking_quality)
        print_arm_tracking(
            "muñecas caminando",
            c.arm_motion_samples.get("walking", []),
        )

        print("  ahora comando cero: tiene que frenar y quedarse...")
        c.change_motion_capture_phase("braking")
        c.spin_for(20.0)
        braking_quality = motion_quality_metrics(
            c.motion_samples["braking"]
        )
        x2, y2, z2, yaw2 = c.pose
        despues_de_frenar = math.hypot(x2 - x1, y2 - y1)
        giro = abs(math.degrees(math.atan2(
            math.sin(yaw2 - yaw0), math.cos(yaw2 - yaw0)
        )))
        print(f"  freno:   altura {z2:.3f} m, siguio {despues_de_frenar:.2f} m mas, "
              f"giro {giro:.0f} grados en total")
        print_motion_quality("torso al frenar", braking_quality)
        print_arm_tracking(
            "muñecas al frenar",
            c.arm_motion_samples.get("braking", []),
        )
    finally:
        c.stop_motion_capture()
        c.release_test_mobility("prueba de caminar terminada")

    if z2 <= STANDING_HEIGHT_MIN:
        return veredicto(False, f"termino en el piso (altura {z2:.3f} m)")
    if recorrido < 0.3:
        return veredicto(False, f"casi no avanzo ({recorrido:.2f} m): "
                                f"la orden no esta llegando a las piernas")
    if path_angle > WALK_MAX_PATH_ANGLE_DEG:
        return veredicto(
            False,
            f"camino en diagonal: {path_angle:.1f} grados fuera de la recta "
            f"(máximo {WALK_MAX_PATH_ANGLE_DEG:.1f})",
        )
    if walk_yaw_error > WALK_MAX_YAW_ERROR_DEG:
        return veredicto(
            False,
            f"giró el cuerpo {walk_yaw_error:.1f} grados mientras caminaba "
            f"(máximo {WALK_MAX_YAW_ERROR_DEG:.1f})",
        )
    if despues_de_frenar > WALK_MAX_BRAKE_M:
        return veredicto(
            False,
            f"siguió {despues_de_frenar:.2f} m después de frenar "
            f"(máximo {WALK_MAX_BRAKE_M:.2f} m)",
        )
    return veredicto(
        True,
        f"avanzó {forward:.2f} m, se desvió {lateral:+.2f} m y frenó en "
        f"{despues_de_frenar:.2f} m",
    )


def check_turn(c: Checker) -> bool:
    """Peldaño 3: una orden positiva produce un giro positivo medido."""
    print("\n=== PELDAÑO 3: el signo del giro coincide con ROS? ===")
    print(
        f"  Ordeno +{TURN_SPEED:.1f} rad/s durante {TURN_DURATION_S:.0f} s "
        "de reloj y mido el cuerpo.\n"
    )

    c.reset_robot()
    if not c.acquire_test_mobility():
        return veredicto(
            False,
            f"el árbitro no concedió TEST; dueño actual: {c.mobility_owner}",
        )
    _, _, z0, yaw0 = c.pose
    try:
        c.drive(TURN_DURATION_S, vyaw=TURN_SPEED)
        c.spin_for(2.0)
        _, _, z1, yaw1 = c.pose
    finally:
        c.release_test_mobility("prueba de signo de giro terminada")

    yaw_change = math.degrees(
        math.atan2(
            math.sin(yaw1 - yaw0),
            math.cos(yaw1 - yaw0),
        )
    )
    if z1 <= STANDING_HEIGHT_MIN:
        return veredicto(False, f"se cayó durante el giro: altura {z1:.3f} m")
    if yaw_change < TURN_MIN_RESPONSE_DEG:
        direction = "sentido contrario" if yaw_change < 0 else "casi no giró"
        return veredicto(
            False,
            f"orden positiva, respuesta {yaw_change:+.1f}° ({direction})",
        )
    return veredicto(
        True,
        f"orden positiva, respuesta {yaw_change:+.1f}°, altura "
        f"{z0:.3f}->{z1:.3f} m",
    )


def check_goto(c: Checker) -> bool:
    """Peldaño 4: la navegacion lleva al robot a un punto y avisa que llego."""
    print("\n=== PELDAÑO 4: llega a un punto y avisa que llego? ===")
    print(f"  Objetivo: {GOTO_DISTANCE} m hacia adelante de donde esta.")
    print("  Ahora si interviene la navegacion. Hasta 3 minutos de reloj.\n")

    c.reset_robot()
    x0, y0, z0, yaw0 = c.pose
    gx = x0 + GOTO_DISTANCE * math.cos(yaw0)
    gy = y0 + GOTO_DISTANCE * math.sin(yaw0)
    print(f"  arranca en ({x0:.2f}, {y0:.2f}) -> objetivo ({gx:.2f}, {gy:.2f})")

    c.nav = None
    g = PoseStamped()
    g.header.frame_id = "map"
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


CHECKS = {
    "authority": check_authority,
    "stand": check_stand,
    "walk": check_walk,
    "turn": check_turn,
    "goto": check_goto,
}


def main():
    cual = sys.argv[1] if len(sys.argv) > 1 else "all"
    if cual not in CHECKS and cual != "all":
        print(f"uso: checks.py {{{'|'.join(CHECKS)}|all}}")
        return 1

    try:
        payload_kg = float(sys.argv[2]) if len(sys.argv) > 2 else None
    except ValueError:
        print("la carga debe expresarse en kilogramos")
        return 1
    if payload_kg is not None and (payload_kg < 0.0 or payload_kg > 3.0):
        print("la carga debe estar entre 0 y 3 kg")
        return 1

    rclpy.init()
    c = Checker(payload_kg=payload_kg)
    try:
        if not c.wait_for_pose():
            print("\n  NO LLEGA /g1/odom: el robot no esta corriendo.\n")
            return 1
        # Con el simulador a RTF ~0,23 puede pasar más de medio segundo de
        # pared hasta el siguiente estado. Esperar evita rechazar una prueba
        # por el último mensaje "frozen" que quedó en tránsito.
        c.spin_for(2.0)
        if c.robot_mode != "active":
            print(
                "\n  ROBOT NO ACTIVO: la prueba no vale si está congelado. "
                "Ejecutá primero: bash run_demo.sh start\n"
            )
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
        if c.mobility_owner == "test":
            c.request_mobility("release", "banco de pruebas detenido")
        elif c.mobility_owner == "manual":
            c.request_mobility(
                "release",
                "banco de pruebas detenido",
                source="manual",
                requester="authority_check",
            )
        c.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
