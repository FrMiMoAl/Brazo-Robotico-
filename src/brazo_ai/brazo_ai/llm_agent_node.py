#!/usr/bin/env python3
"""Nodo ROS 2 para la planificacion simbolica con LLM (OllamaBackend) y RobotPlan.

Convierte instrucciones humanas (/user_command) + estado de la escena (/scene_state)
en un plan JSON estructurado publicado en /llm_plan.

El LLM SOLO planifica acciones de alto nivel. Nunca genera angulos, PWM ni topicos
de control directo.
"""

import json
import os
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .llm_backend import OllamaBackend
from .plan_schema import RobotPlan
from . import plan_utils

FALLBACK_SYSTEM_PROMPT = """You are a symbolic planner for a robotic arm.
Never generate motor commands or coordinates.
Use only objects available in the scene.
Return an abort plan when the request is unsafe or impossible.
"""


import time
from .validation_engine import ValidationEngine

class LlmAgentNode(Node):
    def __init__(self) -> None:
        super().__init__("llm_agent_node")

        self.declare_parameter("model", "qwen3:4b-instruct")
        self.declare_parameter("model_revision", "latest")
        self.declare_parameter("quantization", "q4_0")
        self.declare_parameter("backend_version", "ollama_0.5.7")
        self.declare_parameter("temperature", 0.0)
        self.declare_parameter("seed", 42)
        self.declare_parameter("ollama_host", "http://localhost:11434")
        self.declare_parameter("use_llm_api", True)
        self.declare_parameter("prompt_file", "")
        self.declare_parameter("default_place_zone", "drop_zone_a")

        self.model_id = self.get_parameter("model").get_parameter_value().string_value
        self.model_revision = self.get_parameter("model_revision").get_parameter_value().string_value
        self.quantization = self.get_parameter("quantization").get_parameter_value().string_value
        self.backend_version = self.get_parameter("backend_version").get_parameter_value().string_value
        self.temperature = float(self.get_parameter("temperature").value)
        self.seed = int(self.get_parameter("seed").value)

        host = self.get_parameter("ollama_host").get_parameter_value().string_value
        self.use_llm_api = bool(self.get_parameter("use_llm_api").value)
        self.default_place_zone = self.get_parameter("default_place_zone").value
        self.prompt_file = self.get_parameter("prompt_file").value

        self.current_scene: dict = {}
        self.backend: Optional[OllamaBackend] = None
        self.validation_engine = ValidationEngine()

        if self.use_llm_api:
            self._init_backend(self.model_id, host)

        # Suscripciones
        self.create_subscription(
            String,
            "/scene_state",
            self.scene_callback,
            10,
        )

        self.create_subscription(
            String,
            "/user_command",
            self.command_callback,
            10,
        )

        # Publicador de planes
        self.plan_publisher = self.create_publisher(
            String,
            "/llm_plan",
            10,
        )

        self.get_logger().info(
            f"llm_agent_node iniciado. use_llm_api={self.use_llm_api}, model={self.model_id}, host={host}"
        )

    def _init_backend(self, model: str, host: str) -> None:
        try:
            self.backend = OllamaBackend(model=model, host=host)
            self.get_logger().info(f"OllamaBackend inicializado con modelo '{model}' en '{host}'")
        except Exception as exc:
            self.get_logger().error(
                f"No se pudo inicializar OllamaBackend ({exc}). "
                "Cayendo a modo determinista."
            )
            self.use_llm_api = False
            self.backend = None

    def scene_callback(self, msg: String) -> None:
        try:
            self.current_scene = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error("scene_state recibido no es JSON valido.")

    def command_callback(self, msg: String) -> None:
        command_text = msg.data.strip()
        self.get_logger().info(f"Comando recibido en /user_command: '{command_text}'")

        if self.use_llm_api and self.backend is not None:
            self._process_command_with_llm(command_text)
        else:
            self._process_command_deterministic(command_text)

    def _process_command_with_llm(self, command_text: str) -> None:
        t_start = time.time()
        if not self.current_scene:
            self.get_logger().warn("No hay scene_state disponible. Publicando abort.")
            self.publish_abort("scene_unavailable")
            return

        user_prompt = (
            "SCENE:\n"
            f"{json.dumps(self.current_scene, indent=2)}\n\n"
            "USER COMMAND:\n"
            f"{command_text}"
        )

        system_prompt = self.load_system_prompt()

        try:
            result = self.backend.generate_plan(
                system_prompt=system_prompt,
                user_prompt=user_prompt,
            )
            t_llm_s = result.latency_s

            t_val_start = time.time()
            val_res = self.validation_engine.evaluate_all(result.raw_response, self.current_scene)
            t_val_s = time.time() - t_val_start
            t_total_s = time.time() - t_start

            # Construir la estructura final para /llm_plan
            # Si hay algun bloqueo en las 5 capas, la tarea de ejecucion es "abort"
            is_blocked = (val_res.first_blocking_layer != "none")
            task_exec = "abort" if is_blocked else (val_res.plan_dict.get("task") if val_res.plan_dict else "unknown")

            output_payload = {
                "task": task_exec,
                "reason": val_res.first_blocking_layer,
                "telemetry": {
                    "plan_raw": result.raw_response,
                    "parse_ok": val_res.parse_ok,
                    "parse_error": val_res.parse_error,
                    "schema_ok": val_res.schema_ok,
                    "schema_error": val_res.schema_error,
                    "grounding_ok": val_res.grounding_ok,
                    "grounding_error": val_res.grounding_error,
                    "reach_ok": val_res.reach_ok,
                    "reach_error": val_res.reach_error,
                    "guard_ok": val_res.guard_ok,
                    "guard_error": val_res.guard_error,
                    "first_blocking_layer": val_res.first_blocking_layer,
                    "t_llm_s": t_llm_s,
                    "t_validation_s": t_val_s,
                    "t_total_s": t_total_s,
                    "model_id": self.model_id,
                    "model_revision": self.model_revision,
                    "quantization": self.quantization,
                    "backend_version": self.backend_version,
                    "temperature": self.temperature,
                    "seed": self.seed,
                },
                "plan": val_res.plan_dict if val_res.plan_dict else {},
            }

            output = String()
            output.data = json.dumps(output_payload)
            self.plan_publisher.publish(output)

            self.get_logger().info(
                f"Plan publicado exitosamente en /llm_plan | "
                f"blocking_layer={val_res.first_blocking_layer} t_llm={t_llm_s:.3f}s t_val={t_val_s:.3f}s"
            )

        except Exception as exc:
            self.get_logger().error(f"Error durante la inferencia LLM: {exc}")
            self.publish_abort("llm_failure")

    def _process_command_deterministic(self, command_text: str) -> None:
        t_start = time.time()
        plan_dict = plan_utils.parse_user_command(command_text, self.default_place_zone)
        plan_raw = json.dumps(plan_dict)

        val_res = self.validation_engine.evaluate_all(plan_raw, self.current_scene)
        t_total_s = time.time() - t_start

        is_blocked = (val_res.first_blocking_layer != "none")
        task_exec = "abort" if is_blocked else plan_dict.get("task", "unknown")

        output_payload = {
            "task": task_exec,
            "reason": val_res.first_blocking_layer,
            "telemetry": {
                "plan_raw": plan_raw,
                "parse_ok": val_res.parse_ok,
                "parse_error": val_res.parse_error,
                "schema_ok": val_res.schema_ok,
                "schema_error": val_res.schema_error,
                "grounding_ok": val_res.grounding_ok,
                "grounding_error": val_res.grounding_error,
                "reach_ok": val_res.reach_ok,
                "reach_error": val_res.reach_error,
                "guard_ok": val_res.guard_ok,
                "guard_error": val_res.guard_error,
                "first_blocking_layer": val_res.first_blocking_layer,
                "t_llm_s": 0.0,
                "t_validation_s": t_total_s,
                "t_total_s": t_total_s,
                "model_id": "deterministic_rules",
                "model_revision": "1.0",
                "quantization": "none",
                "backend_version": "rules_engine",
                "temperature": 0.0,
                "seed": 0,
            },
            "plan": plan_dict,
        }

        output = String()
        output.data = json.dumps(output_payload)
        self.plan_publisher.publish(output)
        self.get_logger().info(f"Plan determinista publicado en /llm_plan: {task_exec}")

    def publish_abort(self, reason: str) -> None:
        abort_plan = RobotPlan.create_abort(reason=reason)
        plan_dict = json.loads(abort_plan.model_dump_json())
        output_payload = {
            "task": "abort",
            "reason": reason,
            "telemetry": {
                "plan_raw": json.dumps(plan_dict),
                "parse_ok": False,
                "parse_error": reason,
                "schema_ok": False,
                "schema_error": reason,
                "grounding_ok": False,
                "grounding_error": reason,
                "reach_ok": False,
                "reach_error": reason,
                "guard_ok": False,
                "guard_error": reason,
                "first_blocking_layer": "parse",
                "t_llm_s": 0.0,
                "t_validation_s": 0.0,
                "t_total_s": 0.0,
                "model_id": self.model_id if hasattr(self, "model_id") else "unknown",
                "model_revision": "latest",
                "quantization": "none",
                "backend_version": "unknown",
                "temperature": 0.0,
                "seed": 0,
            },
            "plan": plan_dict,
        }
        output = String()
        output.data = json.dumps(output_payload)
        self.plan_publisher.publish(output)
        self.get_logger().info(f"Abort publicado en /llm_plan: {reason}")

    def load_system_prompt(self) -> str:
        # 1. Si hay parametro prompt_file explicito
        if self.prompt_file and os.path.isfile(self.prompt_file):
            try:
                with open(self.prompt_file, "r", encoding="utf-8") as f:
                    return f.read()
            except Exception as e:
                self.get_logger().warn(f"No se pudo leer prompt_file '{self.prompt_file}': {e}")

        # 2. Intentar buscar planner_prompt.txt en el paquete instalado o relativo
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg_share = get_package_share_directory("brazo_ai")
            prompt_path = os.path.join(pkg_share, "config", "planner_prompt.txt")
            if os.path.isfile(prompt_path):
                with open(prompt_path, "r", encoding="utf-8") as f:
                    return f.read()
        except Exception:
            pass

        # 3. Intentar ruta de desarrollo local si existe
        dev_prompt_path = os.path.join(
            os.path.dirname(__file__), "..", "config", "planner_prompt.txt"
        )
        if os.path.isfile(dev_prompt_path):
            try:
                with open(dev_prompt_path, "r", encoding="utf-8") as f:
                    return f.read()
            except Exception:
                pass

        # 4. Fallback por defecto
        return FALLBACK_SYSTEM_PROMPT


# Alias para retrocompatibilidad
LLMAgentNode = LlmAgentNode


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LlmAgentNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
