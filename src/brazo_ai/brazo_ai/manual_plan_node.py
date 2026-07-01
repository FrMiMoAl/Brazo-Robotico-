#!/usr/bin/env python3
"""Genera planes JSON sin LLM, para pruebas manuales.

Dos modos de uso:

1) Parametro 'task' al lanzar el nodo (publica un plan una vez al iniciar):

   ros2 run brazo_ai manual_plan_node --ros-args -p task:=pick_red

2) Por topico, igual que llm_agent_node en modo determinista:

   ros2 topic pub /user_command std_msgs/msg/String "{data: 'agarra el objeto rojo'}" -1

manual_plan_node reusa las mismas reglas de validacion que llm_agent_node
(brazo_ai/plan_utils.py), de forma que cualquier plan publicado en
/llm_plan -- venga de aqui, del modo determinista o de la API -- pasa por
el mismo control antes de llegar a task_executor_node.
"""

import json

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from . import plan_utils

# Atajos de 'task' para pruebas rapidas por parametro.
TASK_SHORTCUTS = {
    "pick_red": lambda zone: plan_utils.build_pick_and_place_plan("red", zone),
    "open_gripper": lambda zone: {"task": "open_gripper"},
    "close_gripper": lambda zone: {"task": "close_gripper"},
    "go_home": lambda zone: {"task": "go_home"},
    "observe_scene": lambda zone: {"task": "observe_scene"},
    "abort": lambda zone: plan_utils.abort_plan("manual_abort"),
}


class ManualPlanNode(Node):
    def __init__(self):
        super().__init__("manual_plan_node")

        self.declare_parameter("task", "")
        self.declare_parameter("default_place_zone", "drop_zone_a")

        self.default_place_zone = self.get_parameter("default_place_zone").value
        self.latest_scene_state = {}

        self.create_subscription(String, "/user_command", self.user_command_cb, 10)
        self.create_subscription(String, "/scene_state", self.scene_state_cb, 10)
        self.llm_plan_pub = self.create_publisher(String, "/llm_plan", 10)

        task = self.get_parameter("task").value
        if task:
            # Publicar una vez al iniciar. Pequeno delay para dar tiempo a
            # que el publisher se descubra con los subscriptores (executor).
            self.create_timer(0.5, lambda: self._publish_shortcut_once(task))

        self.get_logger().info("manual_plan_node listo (sin LLM, solo para pruebas).")

    def scene_state_cb(self, msg: String):
        try:
            self.latest_scene_state = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error("scene_state recibido no es JSON valido, se ignora.")

    def _publish_shortcut_once(self, task: str):
        self.destroy_timer_safe()
        builder = TASK_SHORTCUTS.get(task)
        if builder is None:
            self.get_logger().error(
                f"task='{task}' no reconocida. Opciones: {list(TASK_SHORTCUTS.keys())}"
            )
            plan = plan_utils.abort_plan("unknown_manual_task")
        else:
            plan = builder(self.default_place_zone)
        self._validate_and_publish(plan)

    def destroy_timer_safe(self):
        # create_timer en __init__ se autodestruye despues del primer disparo
        # manualmente para no repetir la publicacion.
        for timer in list(self.timers):
            self.destroy_timer(timer)

    def user_command_cb(self, msg: String):
        plan = plan_utils.parse_user_command(msg.data, self.default_place_zone)
        self._validate_and_publish(plan)

    def _validate_and_publish(self, plan: dict):
        valid, reason = plan_utils.validate_plan(plan, self.latest_scene_state)
        if not valid:
            self.get_logger().warn(f"Plan rechazado por validacion ({reason}): {plan}")
            plan = plan_utils.abort_plan(reason)

        out = String()
        out.data = plan_utils.plan_to_json(plan)
        self.llm_plan_pub.publish(out)
        self.get_logger().info(f"/llm_plan -> {out.data}")


def main(args=None):
    rclpy.init(args=args)
    node = ManualPlanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
