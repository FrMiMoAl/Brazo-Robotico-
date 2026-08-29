#!/usr/bin/env python3
"""Nodo ROS 2 para la planificacion simbolica con LLM (OllamaBackend) y RobotPlan.

Convierte instrucciones humanas (/user_command) + estado de la escena (/scene_state)
en un plan JSON estructurado publicado en /llm_plan.

El LLM SOLO planifica acciones de alto nivel. Nunca genera angulos, PWM ni topicos
de control directo.

CAMBIOS RESPECTO A LA VERSION ANTERIOR
--------------------------------------
1. EXECUTOR: la suscripcion a /scene_state vive en un ReentrantCallbackGroup y
   main() usa MultiThreadedExecutor. Antes, rclpy.spin() con un solo hilo hacia
   que scene_callback NO pudiera correr durante los 2-26 s de inferencia, asi que
   re-leer self.current_scene devolvia el mismo snapshot viejo. El fix de
   frescura era un no-op.
2. TELEMETRIA HONESTA: publish_abort() ya no inventa veredictos. Las capas que no
   se evaluaron se escriben como None ("NA"), no como False, y first_blocking_layer
   ya no se estampa como "parse" cuando el LLM ni siquiera respondio.
3. CAMPO outcome: distingue ok / llm_failure / llm_timeout / scene_unavailable /
   validation_error. Antes todo caia en el mismo saco.
4. TIMEOUT UNIFICADO: un solo parametro inference_timeout_s que se pasa al cliente
   Ollama y se registra como columna. Antes habia un limite en el harness (10 s) y
   otro en el cliente (30 s), y los CSV quedaron incoherentes.
5. AUDITABILIDAD: se registra scene_timestamp_used y scene_age_s, para poder
   detectar el bug de frescura desde los CSV en vez de por inferencia indirecta.
6. dry_run por defecto FALSE. Un default permisivo en un chequeo de seguridad es
   lo contrario de fail-safe; el benchmark debe pedir dry_run:=true explicitamente.
"""

import copy
import csv
import datetime
import json
import os
import time
from typing import Any, Dict, Optional

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from .llm_backend import OllamaBackend
from .plan_schema import RobotPlan
from .validation_engine import ValidationEngine
from . import plan_utils

FALLBACK_SYSTEM_PROMPT = """You are a symbolic task planner for a 4-DOF robotic arm.
Generate ONLY a valid JSON plan matching the requested schema.

STRICT RULES:
1. Use EXACT object "id" from the SCENE (e.g. "green_0", "red_0"). Never invent IDs.
2. Use EXACT zone names from the SCENE (e.g. "home", "drop_zone_a", "drop_zone_b").
3. DISTINGUISH INTENT:
   - For GRASP/PICK commands ("agarra", "toma", "levanta", "recoge", "pick", "grab", "hold"), generate task "pick" (3 steps: approach, grasp, lift). Do NOT add a place step.
   - For MOVE/TRANSFER commands ("mueve a", "lleva a", "traslada a", "coloca en", "move to", "transfer to"), generate task "pick_and_place" (4 steps: approach, grasp, lift, place).
   - For HOME commands ("ve a home", "inicio", "descanso", "go home"), generate task "home".
4. Allowed actions: approach, grasp, lift, place, home, abort.
5. Only generate an abort task if the requested object is NOT in the scene, or if the command is completely unrelated/dangerous.

Schema for pick (grasp and lift only, no place):
{
  "task": "pick",
  "target_object_id": "green_0",
  "steps": [
    {"action": "approach", "object_id": "green_0", "zone_id": "home"},
    {"action": "grasp", "object_id": "green_0"},
    {"action": "lift", "object_id": "green_0"}
  ]
}

Schema for pick_and_place (grasp, lift, and place in a destination zone):
{
  "task": "pick_and_place",
  "target_object_id": "green_0",
  "destination_zone_id": "drop_zone_a",
  "steps": [
    {"action": "approach", "object_id": "green_0", "zone_id": "home"},
    {"action": "grasp", "object_id": "green_0"},
    {"action": "lift", "object_id": "green_0"},
    {"action": "place", "object_id": "green_0", "zone_id": "drop_zone_a"}
  ]
}

Schema for home:
{
  "task": "home",
  "steps": [
    {"action": "home", "zone_id": "home"}
  ]
}

Reply with ONLY raw JSON, no extra text.
"""

# Valores posibles del campo outcome. Separado de first_blocking_layer porque
# "el LLM no respondio" no es lo mismo que "una capa rechazo el plan".
OUTCOME_OK = "ok"
OUTCOME_LLM_FAILURE = "llm_failure"
OUTCOME_LLM_TIMEOUT = "llm_timeout"
OUTCOME_SCENE_UNAVAILABLE = "scene_unavailable"
OUTCOME_VALIDATION_ERROR = "validation_error"


def _es_timeout(exc: BaseException) -> bool:
    """Detecta timeouts del cliente HTTP sin acoplarse a httpx."""
    nombre = type(exc).__name__.lower()
    return "timeout" in nombre or "timeout" in str(exc).lower()


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
        # Fail-safe: el chequeo de frescura esta ACTIVO salvo peticion explicita.
        self.declare_parameter("dry_run", False)
        # Fuente unica de verdad del timeout de inferencia.
        self.declare_parameter("inference_timeout_s", 30.0)

        self.declare_parameter("workspace_x_min", -0.50)
        self.declare_parameter("workspace_x_max", 0.80)
        self.declare_parameter("workspace_y_min", -0.60)
        self.declare_parameter("workspace_y_max", 0.60)
        self.declare_parameter("workspace_z_min", -0.20)
        self.declare_parameter("workspace_z_max", 0.90)
        self.declare_parameter("max_step_m", 0.60)
        self.declare_parameter("csv_log_path", "metricas_experimentos_llm.csv")

        self.model_id = self.get_parameter("model").get_parameter_value().string_value
        self.model_revision = self.get_parameter("model_revision").get_parameter_value().string_value
        self.quantization = self.get_parameter("quantization").get_parameter_value().string_value
        self.backend_version = self.get_parameter("backend_version").get_parameter_value().string_value
        self.temperature = float(self.get_parameter("temperature").value)
        self.seed = int(self.get_parameter("seed").value)
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.inference_timeout_s = float(self.get_parameter("inference_timeout_s").value)

        host = self.get_parameter("ollama_host").get_parameter_value().string_value
        self.use_llm_api = bool(self.get_parameter("use_llm_api").value)
        self.default_place_zone = self.get_parameter("default_place_zone").value
        self.prompt_file = self.get_parameter("prompt_file").value
        self.last_command_text = ""

        workspace_limits = {
            "workspace_x_min": float(self.get_parameter("workspace_x_min").value),
            "workspace_x_max": float(self.get_parameter("workspace_x_max").value),
            "workspace_y_min": float(self.get_parameter("workspace_y_min").value),
            "workspace_y_max": float(self.get_parameter("workspace_y_max").value),
            "workspace_z_min": float(self.get_parameter("workspace_z_min").value),
            "workspace_z_max": float(self.get_parameter("workspace_z_max").value),
            "max_step_m": float(self.get_parameter("max_step_m").value),
        }

        self.current_scene: Dict[str, Any] = {}
        self.backend: Optional[OllamaBackend] = None

        # dry_run tiene UNA sola fuente de verdad: el parametro del nodo.
        # El motor la recibe en cada llamada, no se duplica como estado.
        self.validation_engine = ValidationEngine(
            workspace_limits=workspace_limits,
            is_dry_run=self.dry_run
        )

        if self.use_llm_api:
            self._init_backend(self.model_id, host)

        # --- Callback groups -------------------------------------------------
        # La escena DEBE poder actualizarse mientras el comando esta bloqueado en
        # inferencia. Sin esto, re-leer current_scene despues del LLM no sirve.
        self._cb_escena = ReentrantCallbackGroup()
        self._cb_comando = MutuallyExclusiveCallbackGroup()

        self.create_subscription(
            String, "/scene_state", self.scene_callback, 10,
            callback_group=self._cb_escena,
        )
        self.create_subscription(
            String, "/user_command", self.command_callback, 10,
            callback_group=self._cb_comando,
        )

        self.plan_publisher = self.create_publisher(String, "/llm_plan", 10)

        self.get_logger().info(
            f"llm_agent_node iniciado. use_llm_api={self.use_llm_api}, "
            f"model={self.model_id}, host={host}, dry_run={self.dry_run}, "
            f"inference_timeout_s={self.inference_timeout_s}"
        )

    # ------------------------------------------------------------------
    def _init_backend(self, model: str, host: str) -> None:
        try:
            self.backend = OllamaBackend(
                model=model, host=host, timeout_s=self.inference_timeout_s
            )
            self.get_logger().info(
                f"OllamaBackend inicializado con '{model}' en '{host}' "
                f"(timeout={self.inference_timeout_s}s)"
            )
        except Exception as exc:
            self.get_logger().error(
                f"No se pudo inicializar OllamaBackend ({exc}). Cayendo a modo determinista."
            )
            self.use_llm_api = False
            self.backend = None

    def scene_callback(self, msg: String) -> None:
        try:
            escena = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error("scene_state recibido no es JSON valido.")
            return
        if not isinstance(escena, dict):
            self.get_logger().error("scene_state no es un objeto JSON.")
            return
        # Reasignacion atomica: los lectores ven la version vieja o la nueva
        # completa, nunca una mezcla.
        self.current_scene = escena

    def command_callback(self, msg: String) -> None:
        command_text = msg.data.strip()
        self.last_command_text = command_text
        self.get_logger().info(f"Comando recibido en /user_command: '{command_text}'")
        if self.use_llm_api and self.backend is not None:
            self._process_command_with_llm(command_text)
        else:
            self._process_command_deterministic(command_text)

    # ------------------------------------------------------------------
    # Telemetria
    # ------------------------------------------------------------------
    def _metadata(self) -> Dict[str, Any]:
        return {
            "model_id": self.model_id,
            "model_revision": self.model_revision,
            "quantization": self.quantization,
            "backend_version": self.backend_version,
            "temperature": self.temperature,
            "seed": self.seed,
            "dry_run": self.dry_run,
            "inference_timeout_s": self.inference_timeout_s,
        }

    @staticmethod
    def _edad_escena(escena: Optional[Dict[str, Any]]) -> Dict[str, Any]:
        ts = (escena or {}).get("timestamp")
        if isinstance(ts, (int, float)) and not isinstance(ts, bool):
            return {"scene_timestamp_used": ts, "scene_age_s": time.time() - ts}
        return {"scene_timestamp_used": None, "scene_age_s": None}

    def _telemetria_desde_validacion(self, val_res) -> Dict[str, Any]:
        return {
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
            "layers_not_evaluated": ",".join(val_res.layers_not_evaluated),
            "plan_valid": val_res.plan_valid,
        }

    @staticmethod
    def _telemetria_sin_validacion(motivo: str) -> Dict[str, Any]:
        return {
            "parse_ok": None, "parse_error": motivo,
            "schema_ok": None, "schema_error": motivo,
            "grounding_ok": None, "grounding_error": motivo,
            "reach_ok": None, "reach_error": motivo,
            "guard_ok": None, "guard_error": motivo,
            "first_blocking_layer": None,
            "layers_not_evaluated": "parse,schema,grounding,reachability,guard",
            "plan_valid": False,
        }

    # ------------------------------------------------------------------
    # Guardado CSV Automatico
    # ------------------------------------------------------------------
    def _guardar_csv(self, payload: Dict[str, Any]) -> None:
        csv_path = self.get_parameter("csv_log_path").get_parameter_value().string_value
        if not csv_path:
            return

        if not os.path.isabs(csv_path):
            csv_path = os.path.join(os.path.expanduser("~"), "Desktop", "Brazo-Robotico-", csv_path)

        tele = payload.get("telemetry", {})
        row = {
            "timestamp": datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "command": self.last_command_text,
            "task": payload.get("task", ""),
            "plan_valid": tele.get("plan_valid", False),
            "first_blocking_layer": tele.get("first_blocking_layer", "none"),
            "parse_ok": tele.get("parse_ok", ""),
            "parse_error": tele.get("parse_error", ""),
            "schema_ok": tele.get("schema_ok", ""),
            "schema_error": tele.get("schema_error", ""),
            "grounding_ok": tele.get("grounding_ok", ""),
            "grounding_error": tele.get("grounding_error", ""),
            "reach_ok": tele.get("reach_ok", ""),
            "reach_error": tele.get("reach_error", ""),
            "guard_ok": tele.get("guard_ok", ""),
            "guard_error": tele.get("guard_error", ""),
            "t_llm_s": f"{tele.get('t_llm_s', 0.0):.4f}" if tele.get('t_llm_s') is not None else "",
            "t_validation_s": f"{tele.get('t_validation_s', 0.0):.5f}" if tele.get('t_validation_s') is not None else "",
            "t_total_s": f"{tele.get('t_total_s', 0.0):.4f}" if tele.get('t_total_s') is not None else "",
            "prompt_tokens": tele.get("prompt_tokens", ""),
            "output_tokens": tele.get("output_tokens", ""),
            "model_id": tele.get("model_id", self.model_id),
            "outcome": tele.get("outcome", ""),
            "scene_age_s": f"{tele.get('scene_age_s', 0.0):.3f}" if tele.get('scene_age_s') is not None else "",
            "plan_raw": str(tele.get("plan_raw", "")).replace("\n", " ").strip(),
        }

        file_exists = os.path.isfile(csv_path)
        try:
            os.makedirs(os.path.dirname(csv_path), exist_ok=True)
            with open(csv_path, "a", newline="", encoding="utf-8") as f:
                writer = csv.DictWriter(f, fieldnames=list(row.keys()))
                if not file_exists:
                    writer.writeheader()
                writer.writerow(row)
            self.get_logger().info(f"📊 [CSV REGISTRADO] -> {csv_path} (blocking={row['first_blocking_layer']}, valid={row['plan_valid']}, t_total={row['t_total_s']}s)")
        except Exception as e:
            self.get_logger().warn(f"No se pudo escribir en CSV '{csv_path}': {e}")

    def _publicar(self, payload: Dict[str, Any]) -> None:
        msg = String()
        msg.data = json.dumps(payload)
        self.plan_publisher.publish(msg)
        self._guardar_csv(payload)

    # ------------------------------------------------------------------
    # Camino LLM
    # ------------------------------------------------------------------
    def _process_command_with_llm(self, command_text: str) -> None:
        t_start = time.time()

        if not self.current_scene:
            self.get_logger().warn("No hay scene_state disponible. Publicando abort.")
            self.publish_abort(OUTCOME_SCENE_UNAVAILABLE, t_start=t_start)
            return

        # Copia profunda: el snapshot del prompt no debe poder mutar si el
        # subscriber reemplaza la escena a mitad de camino.
        prompt_scene = copy.deepcopy(self.current_scene)

        user_prompt = (
            "SCENE:\n"
            f"{json.dumps(prompt_scene, indent=2)}\n\n"
            "USER COMMAND:\n"
            f"{command_text}"
        )
        system_prompt = self.load_system_prompt()

        # --- Inferencia ---------------------------------------------------
        try:
            result = self.backend.generate_plan(
                system_prompt=system_prompt, user_prompt=user_prompt
            )
        except Exception as exc:
            outcome = OUTCOME_LLM_TIMEOUT if _es_timeout(exc) else OUTCOME_LLM_FAILURE
            self.get_logger().error(f"Inferencia fallida ({outcome}): {exc}")
            self.publish_abort(outcome, t_start=t_start, detalle=str(exc))
            return

        t_llm_s = result.latency_s

        # --- Seleccion del snapshot a validar -------------------------------
        # dry-run: escena estatica, se valida contra la misma foto del prompt y
        #          se exime la frescura.
        # real:    se re-consulta la escena mas reciente. Esto SOLO funciona si
        #          scene_callback pudo correr durante la inferencia, de ahi el
        #          ReentrantCallbackGroup y el MultiThreadedExecutor.
        val_scene = prompt_scene if self.dry_run else copy.deepcopy(self.current_scene)

        # --- Validacion ------------------------------------------------------
        t_val_start = time.time()
        try:
            val_res = self.validation_engine.evaluate_all(
                result.raw_response, val_scene, is_dry_run=self.dry_run
            )
        except Exception as exc:
            self.get_logger().error(f"Error en el motor de validacion: {exc}")
            self.publish_abort(OUTCOME_VALIDATION_ERROR, t_start=t_start,
                               detalle=str(exc), t_llm_s=t_llm_s,
                               plan_raw=result.raw_response)
            return
        t_val_s = time.time() - t_val_start
        t_total_s = time.time() - t_start

        # El veredicto de ejecucion se ancla en plan_valid, no en
        # first_blocking_layer, porque con tri-estado "none" tambien aparece
        # cuando una capa quedo sin evaluar.
        task_exec = (
            val_res.plan_dict.get("task", "unknown")
            if val_res.plan_valid and val_res.plan_dict
            else "abort"
        )

        prompt_toks = getattr(result, "input_tokens", None)
        output_toks = getattr(result, "output_tokens", None)

        telemetria: Dict[str, Any] = {"plan_raw": result.raw_response}
        telemetria.update(self._telemetria_desde_validacion(val_res))
        telemetria.update(self._edad_escena(val_scene))
        telemetria.update({
            "outcome": OUTCOME_OK,
            "t_llm_s": t_llm_s,
            "t_validation_s": t_val_s,
            "t_total_s": t_total_s,
            "prompt_tokens": prompt_toks if isinstance(prompt_toks, (int, float)) and not isinstance(prompt_toks, bool) else None,
            "output_tokens": output_toks if isinstance(output_toks, (int, float)) and not isinstance(output_toks, bool) else None,
        })
        telemetria.update(self._metadata())

        self._publicar({
            "task": task_exec,
            "reason": val_res.first_blocking_layer,
            "telemetry": telemetria,
            "plan": val_res.plan_dict or {},
        })

        self.get_logger().info(
            f"Plan publicado | blocking={val_res.first_blocking_layer} "
            f"valid={val_res.plan_valid} t_llm={t_llm_s:.3f}s t_val={t_val_s:.5f}s "
            f"scene_age={telemetria['scene_age_s']}"
        )

    # ------------------------------------------------------------------
    # Camino determinista
    # ------------------------------------------------------------------
    def _process_command_deterministic(self, command_text: str) -> None:
        t_start = time.time()
        plan_dict = plan_utils.parse_user_command(command_text, self.default_place_zone)
        plan_raw = json.dumps(plan_dict)
        val_scene = copy.deepcopy(self.current_scene)

        t_val_start = time.time()
        val_res = self.validation_engine.evaluate_all(
            plan_raw, val_scene, is_dry_run=self.dry_run
        )
        t_val_s = time.time() - t_val_start
        t_total_s = time.time() - t_start

        task_exec = plan_dict.get("task", "unknown") if val_res.plan_valid else "abort"

        telemetria: Dict[str, Any] = {"plan_raw": plan_raw}
        telemetria.update(self._telemetria_desde_validacion(val_res))
        telemetria.update(self._edad_escena(val_scene))
        telemetria.update({
            "outcome": OUTCOME_OK,
            "t_llm_s": 0.0,
            "t_validation_s": t_val_s,
            "t_total_s": t_total_s,
            "prompt_tokens": None,
            "output_tokens": None,
        })
        telemetria.update(self._metadata())
        # El baseline se identifica como tal, sin heredar los metadatos del LLM.
        telemetria.update({
            "model_id": "deterministic_rules",
            "model_revision": "1.0",
            "quantization": "none",
            "backend_version": "rules_engine",
        })

        self._publicar({
            "task": task_exec,
            "reason": val_res.first_blocking_layer,
            "telemetry": telemetria,
            "plan": plan_dict,
        })
        self.get_logger().info(f"Plan determinista publicado: {task_exec}")

    # ------------------------------------------------------------------
    def publish_abort(
        self,
        outcome: str,
        t_start: Optional[float] = None,
        detalle: str = "",
        t_llm_s: float = 0.0,
        plan_raw: str = "",
    ) -> None:
        """Aborto por fallo de infraestructura, NO por rechazo de una capa.

        La distincion importa: estas filas no son mediciones de la pila de
        seguridad y deben excluirse de los denominadores de CRR, UAR y FRR.
        """
        motivo = f"{outcome}:{detalle}" if detalle else outcome
        abort_plan = RobotPlan.create_abort(reason=outcome)
        plan_dict = json.loads(abort_plan.model_dump_json())
        t_total_s = (time.time() - t_start) if t_start is not None else 0.0

        telemetria: Dict[str, Any] = {"plan_raw": plan_raw}
        telemetria.update(self._telemetria_sin_validacion(motivo))
        telemetria.update(self._edad_escena(self.current_scene))
        telemetria.update({
            "outcome": outcome,
            "t_llm_s": t_llm_s,
            "t_validation_s": 0.0,
            "t_total_s": t_total_s,
            "prompt_tokens": None,
            "output_tokens": None,
        })
        telemetria.update(self._metadata())

        self._publicar({
            "task": "abort",
            "reason": outcome,
            "telemetry": telemetria,
            "plan": plan_dict,
        })
        self.get_logger().info(f"Abort publicado: {motivo} (t_total={t_total_s:.3f}s)")

    # ------------------------------------------------------------------
    def load_system_prompt(self) -> str:
        if self.prompt_file and os.path.isfile(self.prompt_file):
            try:
                with open(self.prompt_file, "r", encoding="utf-8") as f:
                    return f.read()
            except Exception as e:
                self.get_logger().warn(f"No se pudo leer prompt_file '{self.prompt_file}': {e}")

        try:
            from ament_index_python.packages import get_package_share_directory
            pkg_share = get_package_share_directory("brazo_ai")
            ruta = os.path.join(pkg_share, "config", "planner_prompt.txt")
            if os.path.isfile(ruta):
                with open(ruta, "r", encoding="utf-8") as f:
                    return f.read()
        except Exception:
            pass

        ruta_dev = os.path.join(os.path.dirname(__file__), "..", "config", "planner_prompt.txt")
        if os.path.isfile(ruta_dev):
            try:
                with open(ruta_dev, "r", encoding="utf-8") as f:
                    return f.read()
            except Exception:
                pass

        return FALLBACK_SYSTEM_PROMPT


LLMAgentNode = LlmAgentNode  # alias de retrocompatibilidad


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LlmAgentNode()

    # MultiThreadedExecutor es OBLIGATORIO, no una optimizacion: con
    # rclpy.spin() la suscripcion de escena queda hambrienta durante la
    # inferencia y la validacion de grounding usa un snapshot caducado.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()