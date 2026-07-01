#!/usr/bin/env python3
"""Unica puerta hacia el brazo real. Valida cada solicitud antes de dejarla pasar.

Entradas:
  /arm/request_target_position  geometry_msgs/PointStamped
  /arm/request_gripper_command  std_msgs/Bool
  /emergency_stop                std_msgs/Bool

Salidas:
  /target_position               geometry_msgs/Point   (SOLO si pasa todas las validaciones)
  /gripper_command                std_msgs/Bool          (SOLO si pasa todas las validaciones)
  /arm/command_status             brazo_interfaces/ArmCommandStatus

Ningun otro nodo de este paquete debe publicar en /target_position ni
/gripper_command. task_executor_node (y el LLM detras de el) solo hablan
con este nodo a traves de los topicos /arm/request_*.

Reglas de bloqueo (ver FASE 10 del spec):
  - frame_id distinto de base_frame
  - punto fuera del workspace configurado
  - emergency_stop == true
  - salto cartesiano respecto al ultimo objetivo aceptado > max_step_m
  - hardware_armed == false y dry_run == false
  - autonomous_enable == false y dry_run == false

Si dry_run == true: nunca se publica a /target_position ni /gripper_command,
solo se registra (log + /arm/command_status) lo que se habria enviado.
"""

import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped, Point

from brazo_interfaces.msg import ArmCommandStatus

STATUS_PREFIX = "safety_guard:"


class SafetyGuardNode(Node):
    def __init__(self):
        super().__init__("safety_guard_node")

        self.declare_parameter("dry_run", True)
        self.declare_parameter("autonomous_enable", False)
        self.declare_parameter("hardware_armed", False)
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("workspace_x_min", 0.05)
        self.declare_parameter("workspace_x_max", 0.35)
        self.declare_parameter("workspace_y_min", -0.25)
        self.declare_parameter("workspace_y_max", 0.25)
        self.declare_parameter("workspace_z_min", 0.03)
        self.declare_parameter("workspace_z_max", 0.35)
        self.declare_parameter("max_step_m", 0.12)

        self.base_frame = self.get_parameter("base_frame").value
        self.emergency_stop = False
        self.last_accepted_point = None  # (x, y, z) del ultimo target aceptado

        self.create_subscription(
            PointStamped, "/arm/request_target_position", self.request_target_cb, 10
        )
        self.create_subscription(
            Bool, "/arm/request_gripper_command", self.request_gripper_cb, 10
        )
        self.create_subscription(Bool, "/emergency_stop", self.emergency_stop_cb, 10)

        self.target_pub = self.create_publisher(Point, "/target_position", 10)
        self.gripper_pub = self.create_publisher(Bool, "/gripper_command", 10)
        self.status_pub = self.create_publisher(ArmCommandStatus, "/arm/command_status", 10)

        self._log_startup_summary()

    def _log_startup_summary(self):
        dry_run = bool(self.get_parameter("dry_run").value)
        autonomous_enable = bool(self.get_parameter("autonomous_enable").value)
        hardware_armed = bool(self.get_parameter("hardware_armed").value)

        self.get_logger().info(
            "safety_guard_node listo. "
            f"dry_run={dry_run} autonomous_enable={autonomous_enable} hardware_armed={hardware_armed}"
        )
        if not dry_run and hardware_armed and autonomous_enable:
            self.get_logger().warn(
                "ATENCION: este nodo puede enviar comandos REALES al brazo "
                "(dry_run=false, hardware_armed=true, autonomous_enable=true)."
            )

    def emergency_stop_cb(self, msg: Bool):
        self.emergency_stop = bool(msg.data)
        if self.emergency_stop:
            self.get_logger().error("EMERGENCY STOP activado. Se bloquean todos los comandos.")

    def publish_status(self, state, success, message, current_target=None):
        status = ArmCommandStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.header.frame_id = self.base_frame
        status.state = state
        status.busy = False
        status.success = success
        status.message = f"{STATUS_PREFIX} {message}"
        if current_target is not None:
            status.current_target = current_target
        self.status_pub.publish(status)
        text = f"[{state}] success={success} {message}"
        # Nota: no se debe alternar entre self.get_logger().info/.warn desde el
        # mismo call-site con una referencia indirecta (log_fn = ... if ... else
        # ...): rclpy cachea la severidad por call-site y lanza
        # "Logger severity cannot be changed between calls". Por eso se usa un
        # if/else explicito con dos call-sites distintos.
        if success:
            self.get_logger().info(text)
        else:
            self.get_logger().warn(text)

    def _gate_checks(self):
        """Chequeos comunes a target y gripper. Devuelve (ok, reason)."""
        dry_run = bool(self.get_parameter("dry_run").value)
        autonomous_enable = bool(self.get_parameter("autonomous_enable").value)
        hardware_armed = bool(self.get_parameter("hardware_armed").value)

        if self.emergency_stop:
            return False, "emergency_stop_active"
        if not dry_run and not hardware_armed:
            return False, "hardware_not_armed"
        if not dry_run and not autonomous_enable:
            return False, "autonomous_not_enabled"
        return True, "ok"

    def request_target_cb(self, msg: PointStamped):
        dry_run = bool(self.get_parameter("dry_run").value)

        if msg.header.frame_id != self.base_frame:
            self.publish_status(
                "REJECTED", False,
                f"frame_id '{msg.header.frame_id}' != '{self.base_frame}'",
                current_target=msg,
            )
            return

        x, y, z = msg.point.x, msg.point.y, msg.point.z
        x_min = float(self.get_parameter("workspace_x_min").value)
        x_max = float(self.get_parameter("workspace_x_max").value)
        y_min = float(self.get_parameter("workspace_y_min").value)
        y_max = float(self.get_parameter("workspace_y_max").value)
        z_min = float(self.get_parameter("workspace_z_min").value)
        z_max = float(self.get_parameter("workspace_z_max").value)

        if not (x_min <= x <= x_max and y_min <= y <= y_max and z_min <= z <= z_max):
            self.publish_status("REJECTED", False, "target_outside_workspace", current_target=msg)
            return

        if self.last_accepted_point is not None:
            lx, ly, lz = self.last_accepted_point
            step = math.sqrt((x - lx) ** 2 + (y - ly) ** 2 + (z - lz) ** 2)
            max_step_m = float(self.get_parameter("max_step_m").value)
            if step > max_step_m:
                self.publish_status(
                    "REJECTED", False,
                    f"cartesian_step {step:.3f}m > max_step_m {max_step_m:.3f}m",
                    current_target=msg,
                )
                return

        ok, reason = self._gate_checks()
        if not ok:
            self.publish_status("REJECTED", False, reason, current_target=msg)
            return

        self.last_accepted_point = (x, y, z)

        if dry_run:
            self.publish_status(
                "DRY_RUN", True,
                f"would publish /target_position ({x:.3f}, {y:.3f}, {z:.3f})",
                current_target=msg,
            )
            return

        point = Point()
        point.x, point.y, point.z = x, y, z
        self.target_pub.publish(point)
        self.publish_status(
            "ACCEPTED", True,
            f"published /target_position ({x:.3f}, {y:.3f}, {z:.3f})",
            current_target=msg,
        )

    def request_gripper_cb(self, msg: Bool):
        dry_run = bool(self.get_parameter("dry_run").value)

        ok, reason = self._gate_checks()
        if not ok:
            self.publish_status("REJECTED", False, reason)
            return

        if dry_run:
            self.publish_status("DRY_RUN", True, f"would publish /gripper_command open={msg.data}")
            return

        self.gripper_pub.publish(msg)
        self.publish_status("ACCEPTED", True, f"published /gripper_command open={msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = SafetyGuardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
