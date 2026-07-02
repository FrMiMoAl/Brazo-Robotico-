#!/usr/bin/env python3
"""Convierte topicos ROS en un JSON semantico que el LLM puede leer.

Entradas:
  /perception/objects_base   brazo_interfaces/Object3DArray
  /joint_states               sensor_msgs/JointState   (opcional)
  /gripper_state               std_msgs/Bool            (opcional)
  /arm_state                   std_msgs/String          (opcional)
  /arm/command_status          brazo_interfaces/ArmCommandStatus (opcional, da busy/state)

Salida:
  /scene_state                 std_msgs/String (JSON)

Publica a frecuencia fija (2 Hz por defecto), no a la tasa de la camara.
"""

import json
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState

from brazo_interfaces.msg import Object3DArray, ArmCommandStatus

ALLOWED_TASKS = [
    "observe_scene",
    "pick_object",
    "pick_and_place",
    "open_gripper",
    "close_gripper",
    "go_home",
    "abort",
]


class SceneStateNode(Node):
    def __init__(self):
        super().__init__("scene_state_node")

        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("camera_frame", "kinect2_depth_optical_frame")
        self.declare_parameter("publish_rate_hz", 2.0)
        self.declare_parameter("hardware_armed", False)

        # Zonas conocidas (deben coincidir con las usadas en task_executor_node,
        # ver parametros homonimos alli; se pasan por launch para mantenerlas en sync)
        self.declare_parameter("zone_home", [0.16, 0.0, 0.20])
        self.declare_parameter("zone_drop_zone_a", [0.18, -0.15, 0.12])
        self.declare_parameter("zone_drop_zone_b", [0.18, 0.15, 0.12])

        self.base_frame = self.get_parameter("base_frame").value
        self.camera_frame = self.get_parameter("camera_frame").value

        self.latest_objects = []
        self.latest_joint_state = None
        self.latest_gripper = "unknown"
        self.latest_arm_state = "idle"
        self.latest_busy = False

        self.create_subscription(Object3DArray, "/perception/objects_base", self.objects_cb, 10)
        self.create_subscription(JointState, "/joint_states", self.joint_state_cb, 10)
        self.create_subscription(Bool, "/gripper_state", self.gripper_state_cb, 10)
        self.create_subscription(String, "/arm_state", self.arm_state_cb, 10)
        self.create_subscription(ArmCommandStatus, "/arm/command_status", self.command_status_cb, 10)

        self.scene_state_pub = self.create_publisher(String, "/scene_state", 10)

        rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(1.0 / rate_hz, self.publish_scene_state)

        self.get_logger().info(f"scene_state_node listo, publicando a {rate_hz} Hz en /scene_state.")

    def objects_cb(self, msg: Object3DArray):
        self.latest_objects = list(msg.objects)

    def joint_state_cb(self, msg: JointState):
        self.latest_joint_state = msg

    def gripper_state_cb(self, msg: Bool):
        self.latest_gripper = "open" if msg.data else "closed"

    def arm_state_cb(self, msg: String):
        self.latest_arm_state = msg.data

    def command_status_cb(self, msg: ArmCommandStatus):
        self.latest_busy = msg.busy
        if msg.state:
            self.latest_arm_state = msg.state

    def zone_dict(self, param_name):
        values = list(self.get_parameter(param_name).value)
        return {"x": float(values[0]), "y": float(values[1]), "z": float(values[2])}

    def publish_scene_state(self):
        hardware_armed = bool(self.get_parameter("hardware_armed").value)

        objects = []
        for obj in self.latest_objects:
            objects.append({
                "id": obj.object_id,
                "class": obj.class_name,
                "confidence": round(float(obj.confidence), 3),
                "reachable": bool(obj.reachable),
                "reason": obj.reason,
                "position_base": {
                    "x": round(float(obj.point.x), 4),
                    "y": round(float(obj.point.y), 4),
                    "z": round(float(obj.point.z), 4),
                },
            })

        state = {
            "timestamp": time.time(),
            "frames": {
                "base": self.base_frame,
                "camera": self.camera_frame,
            },
            "robot": {
                "state": self.latest_arm_state,
                "busy": self.latest_busy,
                "gripper": self.latest_gripper,
                "hardware_armed": hardware_armed,
            },
            "objects": objects,
            "zones": {
                "home": self.zone_dict("zone_home"),
                "drop_zone_a": self.zone_dict("zone_drop_zone_a"),
                "drop_zone_b": self.zone_dict("zone_drop_zone_b"),
            },
            "allowed_tasks": ALLOWED_TASKS,
        }

        msg = String()
        msg.data = json.dumps(state)
        self.scene_state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SceneStateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
