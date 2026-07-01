#!/usr/bin/env python3
"""Convierte instrucciones humanas + scene_state en un plan JSON (/llm_plan).

El LLM (o el modo determinista MVP) SOLO planifica. Nunca publica en
/joint_commands, /target_position ni /gripper_command. Ver
brazo_ai/plan_utils.py para las reglas de validacion compartidas.

Modo por defecto (use_llm_api:=false): traduccion determinista de texto a
plan, sin red ni claves de API.

Modo opcional (use_llm_api:=true): usa la API de Anthropic via la SDK
oficial, leyendo la clave SOLO desde la variable de entorno
ANTHROPIC_API_KEY (nunca hardcodeada). El plan devuelto por la API se
valida igual que en el modo determinista antes de publicarse.
"""

import json
import os

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from . import plan_utils

SYSTEM_PROMPT = """Eres un planificador de alto nivel para un brazo robotico de 4 grados de libertad con Kinect v2 en ROS 2.

Recibiras:
1. Una instruccion humana.
2. Un JSON scene_state con objetos detectados, posiciones en base_link, estado del robot y zonas disponibles.

Tu trabajo:
Convertir la instruccion humana en un plan JSON valido.

Reglas obligatorias:
- No controles articulaciones.
- No generes angulos.
- No generes PWM.
- No generes velocidades.
- No publiques en /joint_commands.
- No publiques en /target_position.
- No publiques en /gripper_command.
- No inventes coordenadas.
- Usa solo objetos presentes en scene_state.
- Usa solo zonas presentes en scene_state.
- Si el objeto no existe, responde abort.
- Si el objeto tiene reachable=false, responde abort.
- Si el robot esta busy=true, responde abort u observe_scene.
- Responde exclusivamente JSON. No expliques.

Tareas permitidas:
- observe_scene
- pick_object
- pick_and_place
- open_gripper
- close_gripper
- go_home
- abort

Formato valido para pick and place:
{
  "task": "pick_and_place",
  "object": {
    "class_name": "red",
    "selection": "largest_reachable"
  },
  "place_zone": "drop_zone_a"
}

Formato valido para abort:
{
  "task": "abort",
  "reason": "explicacion_corta"
}
"""


class LlmAgentNode(Node):
    def __init__(self):
        super().__init__("llm_agent_node")

        self.declare_parameter("use_llm_api", False)
        self.declare_parameter("llm_model", "claude-sonnet-5")
        self.declare_parameter("default_place_zone", "drop_zone_a")

        self.use_llm_api = bool(self.get_parameter("use_llm_api").value)
        self.default_place_zone = self.get_parameter("default_place_zone").value

        self.latest_scene_state = {}
        self._anthropic_client = None

        if self.use_llm_api:
            self._init_llm_client()

        self.create_subscription(String, "/user_command", self.user_command_cb, 10)
        self.create_subscription(String, "/scene_state", self.scene_state_cb, 10)
        self.llm_plan_pub = self.create_publisher(String, "/llm_plan", 10)

        self.get_logger().info(
            f"llm_agent_node listo. use_llm_api={self.use_llm_api}. "
            "Solo publica planes JSON en /llm_plan, nunca comandos directos."
        )

    def _init_llm_client(self):
        api_key = os.environ.get("ANTHROPIC_API_KEY")
        if not api_key:
            self.get_logger().error(
                "use_llm_api=true pero ANTHROPIC_API_KEY no esta definida en el entorno. "
                "Cayendo a modo determinista (use_llm_api se trata como false)."
            )
            self.use_llm_api = False
            return

        try:
            import anthropic
        except ImportError:
            self.get_logger().error(
                "use_llm_api=true pero el paquete 'anthropic' no esta instalado "
                "(pip install anthropic). Cayendo a modo determinista."
            )
            self.use_llm_api = False
            return

        self._anthropic_client = anthropic.Anthropic(api_key=api_key)

    def scene_state_cb(self, msg: String):
        try:
            self.latest_scene_state = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error("scene_state recibido no es JSON valido, se ignora.")

    def user_command_cb(self, msg: String):
        text = msg.data
        self.get_logger().info(f"user_command recibido: '{text}'")

        if self.use_llm_api and self._anthropic_client is not None:
            plan = self._plan_via_llm_api(text)
        else:
            plan = plan_utils.parse_user_command(text, self.default_place_zone)

        valid, reason = plan_utils.validate_plan(plan, self.latest_scene_state)
        if not valid:
            self.get_logger().warn(f"Plan rechazado por validacion ({reason}): {plan}")
            plan = plan_utils.abort_plan(reason)

        out = String()
        out.data = plan_utils.plan_to_json(plan)
        self.llm_plan_pub.publish(out)
        self.get_logger().info(f"/llm_plan -> {out.data}")

    def _plan_via_llm_api(self, user_text: str) -> dict:
        scene_json = json.dumps(self.latest_scene_state)
        user_message = f"Instruccion humana: {user_text}\n\nscene_state:\n{scene_json}"

        model = self.get_parameter("llm_model").value
        try:
            response = self._anthropic_client.messages.create(
                model=model,
                max_tokens=512,
                system=SYSTEM_PROMPT,
                messages=[{"role": "user", "content": user_message}],
            )
            raw_text = "".join(
                block.text for block in response.content if getattr(block, "type", None) == "text"
            )
            return json.loads(raw_text)
        except Exception as e:
            self.get_logger().error(f"Error consultando la API del LLM: {e}")
            return plan_utils.abort_plan("llm_api_error")


def main(args=None):
    rclpy.init(args=args)
    node = LlmAgentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
