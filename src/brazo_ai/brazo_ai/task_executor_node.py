#!/usr/bin/env python3
"""Ejecuta planes de alto nivel (/llm_plan) como una maquina de estados determinista.

Entradas:
  /llm_plan                  std_msgs/String (JSON)
  /perception/objects_base   brazo_interfaces/Object3DArray
  /scene_state                std_msgs/String (JSON, usado para leer zonas)

Salidas (SIEMPRE hacia safety_guard_node, nunca directo al brazo):
  /arm/request_target_position  geometry_msgs/PointStamped
  /arm/request_gripper_command  std_msgs/Bool   (True=abrir, False=cerrar)
  /arm/command_status           brazo_interfaces/ArmCommandStatus

Nota de diseno: el parametro 'dry_run' de este nodo SOLO afecta el texto
de log/estado ("[DRY_RUN] ..."). La decision real de si un comando llega
al hardware la toma exclusivamente safety_guard_node con su propio
'dry_run' (que es la unica puerta hacia /target_position y
/gripper_command). Asi, task_executor_node siempre puede ensayar la
secuencia completa sin tocar el brazo real, incluso si por error se
lanzara con dry_run:=false aqui.
"""

import json
import math
import threading
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool
from geometry_msgs.msg import PointStamped

from brazo_interfaces.msg import Object3DArray, ArmCommandStatus

from .plan_utils import ALLOWED_TASKS

SAFETY_PREFIX = "safety_guard:"


class TaskExecutorNode(Node):
    def __init__(self):
        super().__init__("task_executor_node")

        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("pregrasp_dz", 0.08)
        self.declare_parameter("grasp_dz", 0.015)
        self.declare_parameter("lift_dz", 0.12)
        self.declare_parameter("move_wait_s", 2.0)
        self.declare_parameter("gripper_wait_s", 1.0)
        self.declare_parameter("default_place_zone", "drop_zone_a")
        self.declare_parameter("dry_run", True)
        # Debe coincidir con el max_step_m de safety_guard_node (el launch
        # file pasa el mismo valor a ambos). Se usa para partir movimientos
        # largos (p.ej. de la zona de pick a la zona de place) en varios
        # waypoints intermedios, ya que safety_guard_node rechaza cualquier
        # salto cartesiano mayor a max_step_m.
        self.declare_parameter("max_step_m", 0.12)

        self.base_frame = self.get_parameter("base_frame").value
        self.dry_run = bool(self.get_parameter("dry_run").value)

        self.latest_objects_base = []
        self.latest_scene_state = {}
        self._lock = threading.Lock()
        self._busy = False
        self._last_safety_rejected = False
        self._last_sent_target = None  # (x, y, z), referencia para partir movimientos largos

        self.create_subscription(String, "/llm_plan", self.llm_plan_cb, 10)
        self.create_subscription(Object3DArray, "/perception/objects_base", self.objects_cb, 10)
        self.create_subscription(String, "/scene_state", self.scene_state_cb, 10)
        self.create_subscription(ArmCommandStatus, "/arm/command_status", self.command_status_cb, 10)

        self.request_target_pub = self.create_publisher(PointStamped, "/arm/request_target_position", 10)
        self.request_gripper_pub = self.create_publisher(Bool, "/arm/request_gripper_command", 10)
        self.status_pub = self.create_publisher(ArmCommandStatus, "/arm/command_status", 10)

        self.get_logger().info(
            f"task_executor_node listo (dry_run={self.dry_run}). "
            "Publica solicitudes a safety_guard_node, nunca directo al brazo."
        )

    def objects_cb(self, msg: Object3DArray):
        self.latest_objects_base = list(msg.objects)

    def scene_state_cb(self, msg: String):
        try:
            self.latest_scene_state = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error("scene_state recibido no es JSON valido, se ignora.")

    def command_status_cb(self, msg: ArmCommandStatus):
        # safety_guard_node publica tambien en /arm/command_status. Lo
        # distinguimos por el prefijo del mensaje para saber si rechazo
        # la ultima solicitud mientras esperamos un movimiento.
        if msg.message.startswith(SAFETY_PREFIX) and not msg.success:
            self._last_safety_rejected = True

    def publish_status(self, state, busy, success, message, current_target=None):
        status = ArmCommandStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.header.frame_id = self.base_frame
        status.state = state
        status.busy = busy
        status.success = success
        status.message = message
        if current_target is not None:
            status.current_target = current_target
        self.status_pub.publish(status)
        self.get_logger().info(f"[{state}] busy={busy} success={success} {message}")

    def llm_plan_cb(self, msg: String):
        with self._lock:
            if self._busy:
                self.get_logger().warn("Plan recibido pero el executor ya esta ocupado. Se ignora.")
                self.publish_status("BUSY", True, False, "executor_busy_ignoring_new_plan")
                return
            self._busy = True

        thread = threading.Thread(target=self._run_plan, args=(msg.data,), daemon=True)
        thread.start()

    def _run_plan(self, raw_json):
        try:
            self._execute(raw_json)
        finally:
            with self._lock:
                self._busy = False

    def _execute(self, raw_json):
        self.publish_status("LOAD_PLAN", True, False, "loading plan")
        try:
            plan = json.loads(raw_json)
        except json.JSONDecodeError:
            self.publish_status("ABORTED", False, False, "invalid_json")
            return

        self.publish_status("VALIDATE_PLAN", True, False, f"plan={plan}")
        task = plan.get("task")
        if task not in ALLOWED_TASKS:
            self.publish_status("ABORTED", False, False, "task_not_allowed")
            return

        if task == "abort":
            self.publish_status("DONE", False, True, f"abort: {plan.get('reason', 'no_reason')}")
            return

        if task == "observe_scene":
            self.publish_status("DONE", False, True, "observe_scene_noop")
            return

        if task == "open_gripper":
            self._run_gripper_only(True)
            return

        if task == "close_gripper":
            self._run_gripper_only(False)
            return

        if task == "go_home":
            self._run_go_home()
            return

        if task in ("pick_object", "pick_and_place"):
            self._run_pick(plan, do_place=(task == "pick_and_place"))
            return

        self.publish_status("ABORTED", False, False, "unhandled_task")

    # ------------------------------------------------------------------
    # Primitivas
    # ------------------------------------------------------------------

    def _select_object(self, class_name):
        candidates = [
            o for o in self.latest_objects_base
            if o.class_name == class_name and o.reachable
        ]
        if not candidates:
            return None
        candidates.sort(key=lambda o: (o.bbox_area, o.confidence), reverse=True)
        return candidates[0]

    def _zone_point(self, zone_name):
        zones = self.latest_scene_state.get("zones", {})
        zone = zones.get(zone_name)
        if zone is None:
            return None
        return float(zone["x"]), float(zone["y"]), float(zone["z"])

    def _interpolate_waypoints(self, start, end, max_step_m):
        dx, dy, dz = end[0] - start[0], end[1] - start[1], end[2] - start[2]
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        if dist <= max_step_m or dist == 0.0:
            return [end]
        n = math.ceil(dist / max_step_m)
        return [
            (start[0] + dx * i / n, start[1] + dy * i / n, start[2] + dz * i / n)
            for i in range(1, n + 1)
        ]

    def _send_target(self, x, y, z, state_name):
        """Mueve a (x, y, z), partiendo el trayecto en waypoints <= max_step_m.

        safety_guard_node rechaza cualquier salto cartesiano mayor a
        max_step_m respecto al ultimo objetivo aceptado. Como aqui no hay
        feedback real de la posicion del brazo (FASE 9.6, MVP), se usa el
        ultimo punto solicitado por este nodo como referencia para
        interpolar. Si es el primer movimiento desde que arranco el nodo,
        se envia directo (no hay referencia de origen conocida).
        """
        max_step_m = float(self.get_parameter("max_step_m").value)
        end = (float(x), float(y), float(z))

        if self._last_sent_target is None:
            waypoints = [end]
        else:
            waypoints = self._interpolate_waypoints(self._last_sent_target, end, max_step_m)

        for idx, wp in enumerate(waypoints):
            label = state_name if idx == len(waypoints) - 1 else f"{state_name}_WP{idx + 1}"
            if not self._send_single_target(*wp, label):
                return False
            self._last_sent_target = wp

        return True

    def _send_single_target(self, x, y, z, state_name):
        target = PointStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = self.base_frame
        target.point.x = float(x)
        target.point.y = float(y)
        target.point.z = float(z)

        prefix = "[DRY_RUN] " if self.dry_run else ""
        self.publish_status(
            state_name, True, False,
            f"{prefix}requesting target ({x:.3f}, {y:.3f}, {z:.3f})",
            current_target=target,
        )

        self._last_safety_rejected = False
        self.request_target_pub.publish(target)

        wait_s = float(self.get_parameter("move_wait_s").value)
        elapsed = 0.0
        step = 0.05
        while elapsed < wait_s:
            time.sleep(step)
            elapsed += step
            if self._last_safety_rejected:
                return False
        return True

    def _send_gripper(self, open_gripper: bool, state_name="GRIPPER"):
        prefix = "[DRY_RUN] " if self.dry_run else ""
        self.publish_status(
            state_name, True, False,
            f"{prefix}requesting gripper open={open_gripper}",
        )
        self._last_safety_rejected = False
        msg = Bool()
        msg.data = bool(open_gripper)
        self.request_gripper_pub.publish(msg)
        time.sleep(float(self.get_parameter("gripper_wait_s").value))
        return not self._last_safety_rejected

    def _run_gripper_only(self, open_gripper: bool):
        ok = self._send_gripper(open_gripper, "OPEN_GRIPPER" if open_gripper else "CLOSE_GRIPPER")
        if not ok:
            self.publish_status("ABORTED", False, False, "safety_guard_rejected")
            return
        self.publish_status("DONE", False, True, "gripper_command_done")

    def _run_go_home(self):
        home = self._zone_point("home")
        if home is None:
            self.publish_status("ABORTED", False, False, "zone_home_not_in_scene_state")
            return
        ok = self._send_target(*home, "GO_HOME")
        if not ok:
            self.publish_status("ABORTED", False, False, "safety_guard_rejected")
            return
        self.publish_status("DONE", False, True, "go_home_done")

    def _run_pick(self, plan, do_place: bool):
        self.publish_status("SELECT_OBJECT", True, False, "selecting object")
        obj_info = plan.get("object", {})
        class_name = obj_info.get("class_name")
        if not class_name:
            self.publish_status("ABORTED", False, False, "missing_class_name")
            return

        obj = self._select_object(class_name)

        self.publish_status("VALIDATE_OBJECT_REACHABLE", True, False, f"class={class_name}")
        if obj is None:
            self.publish_status("ABORTED", False, False, "no_reachable_object")
            return

        x, y, z = obj.point.x, obj.point.y, obj.point.z
        pregrasp_dz = float(self.get_parameter("pregrasp_dz").value)
        grasp_dz = float(self.get_parameter("grasp_dz").value)
        lift_dz = float(self.get_parameter("lift_dz").value)

        if not self._send_gripper(True, "OPEN_GRIPPER"):
            self.publish_status("ABORTED", False, False, "safety_guard_rejected")
            return

        if not self._send_target(x, y, z + pregrasp_dz, "MOVE_PREGRASP"):
            self.publish_status("ABORTED", False, False, "safety_guard_rejected")
            return

        if not self._send_target(x, y, z + grasp_dz, "MOVE_GRASP"):
            self.publish_status("ABORTED", False, False, "safety_guard_rejected")
            return

        if not self._send_gripper(False, "CLOSE_GRIPPER"):
            self.publish_status("ABORTED", False, False, "safety_guard_rejected")
            return

        if not self._send_target(x, y, z + lift_dz, "LIFT_OBJECT"):
            self.publish_status("ABORTED", False, False, "safety_guard_rejected")
            return

        if not do_place:
            self.publish_status("DONE", False, True, "pick_object_done")
            return

        place_zone = plan.get("place_zone") or self.get_parameter("default_place_zone").value
        place = self._zone_point(place_zone)
        if place is None:
            self.publish_status("ABORTED", False, False, "place_zone_not_in_scene_state")
            return

        if not self._send_target(*place, "MOVE_TO_PLACE_ZONE"):
            self.publish_status("ABORTED", False, False, "safety_guard_rejected")
            return

        if not self._send_gripper(True, "OPEN_GRIPPER"):
            self.publish_status("ABORTED", False, False, "safety_guard_rejected")
            return

        retreat = (place[0], place[1], place[2] + lift_dz)
        if not self._send_target(*retreat, "RETREAT"):
            self.publish_status("ABORTED", False, False, "safety_guard_rejected")
            return

        home = self._zone_point("home")
        if home is not None:
            self._send_target(*home, "GO_HOME")

        self.publish_status("DONE", False, True, "pick_and_place_done")


def main(args=None):
    rclpy.init(args=args)
    node = TaskExecutorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
