"""Motor de validación desacoplado y sin cortocircuito para el pipeline de IA del brazo robótico.

Permite evaluar cada capa de seguridad de forma independiente sobre el mismo plan
para telemetría y estudios de ablación (A-E), garantizando al mismo tiempo que
cualquier fallo bloquee la ejecución física real del robot.
"""

import json
import math
import time
from dataclasses import dataclass, asdict
from typing import Dict, Any, Tuple, Optional

from .plan_schema import RobotPlan, ActionName

ALLOWED_TASKS = {
    "observe_scene",
    "pick",
    "pick_object",
    "pick_and_place",
    "open_gripper",
    "close_gripper",
    "home",
    "go_home",
    "abort",
}

FORBIDDEN_KEYS = {
    "joint_1", "joint_2", "joint_3", "joint_4",
    "servo_pwm", "pwm", "angle", "angles", "velocity", "velocities",
    "joint_commands", "target_position", "gripper_command",
}


@dataclass
class ValidationResult:
    parse_ok: bool
    parse_error: str
    schema_ok: bool
    schema_error: str
    grounding_ok: bool
    grounding_error: str
    reach_ok: bool
    reach_error: str
    guard_ok: bool
    guard_error: str
    first_blocking_layer: str
    plan_dict: Optional[Dict[str, Any]] = None

    def to_dict(self) -> Dict[str, Any]:
        d = asdict(self)
        # Excluir plan_dict de dict plano de telemetría si es necesario
        return d


class ValidationEngine:
    """Motor de validación multi-capa sin cortocircuito."""

    def __init__(
        self,
        workspace_limits: Optional[Dict[str, float]] = None,
        max_stale_age_s: float = 2.0,
    ) -> None:
        self.workspace_limits = workspace_limits or {
            "workspace_x_min": 0.05,
            "workspace_x_max": 0.35,
            "workspace_y_min": -0.25,
            "workspace_y_max": 0.25,
            "workspace_z_min": 0.03,
            "workspace_z_max": 0.35,
            "max_step_m": 0.12,
        }
        self.max_stale_age_s = max_stale_age_s

    def check_parse(self, plan_raw: str) -> Tuple[bool, str, Optional[Dict[str, Any]]]:
        """Capa 1: Parseo de sintaxis JSON y verificación de claves prohibidas de motor."""
        if not isinstance(plan_raw, str) or not plan_raw.strip():
            return False, "empty_or_non_string_raw", None

        try:
            parsed = json.loads(plan_raw)
        except Exception as exc:
            return False, f"json_syntax_error:{exc}", None

        if not isinstance(parsed, dict):
            return False, "raw_json_is_not_object", None

        # Verificar presencia de claves de motor prohibidas
        found_forbidden = [k for k in parsed.keys() if k in FORBIDDEN_KEYS]
        if found_forbidden:
            return False, f"forbidden_keys_found:{','.join(found_forbidden)}", parsed

        return True, "", parsed

    def check_schema(self, plan_dict: Optional[Dict[str, Any]]) -> Tuple[bool, str]:
        """Capa 2: Validación de esquema Pydantic y reglas estructurales."""
        if not plan_dict or not isinstance(plan_dict, dict):
            return False, "missing_or_invalid_plan_dict"

        task = plan_dict.get("task")
        if not task or task not in ALLOWED_TASKS:
            return False, f"invalid_task:{task}"

        if task == "abort":
            return True, ""

        # Verificar si hay valores NaN o Inf en cualquier campo numérico
        if self._has_nan_or_inf(plan_dict):
            return False, "nan_or_inf_values_detected"

        # Validación específica para tareas pick / pick_and_place
        if task in ("pick", "pick_object", "pick_and_place"):
            target_obj = plan_dict.get("target_object_id")
            obj_dict = plan_dict.get("object")
            class_name = target_obj or (obj_dict.get("class_name") if isinstance(obj_dict, dict) else None)

            if not class_name:
                steps = plan_dict.get("steps", [])
                if isinstance(steps, list):
                    for step in steps:
                        if isinstance(step, dict) and step.get("object_id"):
                            class_name = step.get("object_id")
                            break

            if not class_name:
                return False, "missing_target_object_class"

            if task == "pick_and_place":
                place_zone = plan_dict.get("destination_zone_id") or plan_dict.get("place_zone")
                if not place_zone:
                    steps = plan_dict.get("steps", [])
                    if isinstance(steps, list):
                        for step in steps:
                            if isinstance(step, dict) and step.get("zone_id"):
                                place_zone = step.get("zone_id")
                                break
                if not place_zone:
                    return False, "missing_destination_zone"

        # Validar pasos si se especifica la lista de steps Pydantic
        steps = plan_dict.get("steps")
        if steps is not None:
            if not isinstance(steps, list) or len(steps) == 0:
                return False, "invalid_or_empty_steps_list"
            allowed_actions = {a.value for a in ActionName}
            for idx, step in enumerate(steps):
                if not isinstance(step, dict):
                    return False, f"step_{idx}_not_dict"
                act = step.get("action")
                if not act or act not in allowed_actions:
                    return False, f"step_{idx}_unknown_action:{act}"
                h = step.get("height_m")
                if h is not None:
                    if not isinstance(h, (int, float)) or math.isnan(h) or math.isinf(h) or h < 0.0 or h > 0.15:
                        return False, f"step_{idx}_invalid_height:{h}"

        return True, ""

    def check_grounding(self, plan_dict: Optional[Dict[str, Any]], scene_state: Optional[Dict[str, Any]]) -> Tuple[bool, str]:
        """Capa 3: Anclaje a la escena actual (Grounding de objetos y zonas)."""
        if not scene_state or not isinstance(scene_state, dict):
            return False, "scene_state_unavailable"

        # Verificar frescura de la escena
        ts = scene_state.get("timestamp")
        if ts is not None and isinstance(ts, (int, float)):
            age = time.time() - ts
            if age > self.max_stale_age_s:
                return False, f"stale_scene_state:age={age:.2f}s"

        if not plan_dict or not isinstance(plan_dict, dict):
            return False, "invalid_plan_dict"

        task = plan_dict.get("task")
        if task in ("home", "go_home", "open_gripper", "close_gripper", "observe_scene", "abort"):
            return True, ""

        if task in ("pick", "pick_object", "pick_and_place"):
            target_obj = plan_dict.get("target_object_id")
            obj_dict = plan_dict.get("object")
            class_name = target_obj or (obj_dict.get("class_name") if isinstance(obj_dict, dict) else None)
            if not class_name and isinstance(plan_dict.get("steps"), list):
                for st in plan_dict["steps"]:
                    if isinstance(st, dict) and st.get("object_id"):
                        class_name = st.get("object_id")
                        break

            scene_objects = scene_state.get("objects", [])
            existing_ids_and_classes = set()
            for obj in scene_objects:
                if isinstance(obj, dict):
                    if obj.get("id"):
                        existing_ids_and_classes.add(str(obj.get("id")))
                    if obj.get("class"):
                        existing_ids_and_classes.add(str(obj.get("class")))
                    if obj.get("class_name"):
                        existing_ids_and_classes.add(str(obj.get("class_name")))

            if class_name not in existing_ids_and_classes:
                return False, f"object_not_in_scene:{class_name}"

            if task == "pick_and_place":
                place_zone = plan_dict.get("destination_zone_id") or plan_dict.get("place_zone")
                if not place_zone and isinstance(plan_dict.get("steps"), list):
                    for st in plan_dict["steps"]:
                        if isinstance(st, dict) and st.get("zone_id"):
                            place_zone = st.get("zone_id")
                            break
                scene_zones = scene_state.get("zones", {})
                if isinstance(scene_zones, dict):
                    if place_zone not in scene_zones:
                        return False, f"place_zone_not_in_scene:{place_zone}"

        return True, ""

    def check_reachability(self, plan_dict: Optional[Dict[str, Any]], scene_state: Optional[Dict[str, Any]]) -> Tuple[bool, str]:
        """Capa 4: Chequeo de alcanzabilidad cinemática del objeto en la escena."""
        if not scene_state or not isinstance(scene_state, dict):
            return False, "scene_state_unavailable"

        if not plan_dict or not isinstance(plan_dict, dict):
            return False, "invalid_plan_dict"

        task = plan_dict.get("task")
        if task in ("home", "go_home", "open_gripper", "close_gripper", "observe_scene", "abort"):
            return True, ""

        if task in ("pick", "pick_object", "pick_and_place"):
            target_obj = plan_dict.get("target_object_id")
            obj_dict = plan_dict.get("object")
            class_name = target_obj or (obj_dict.get("class_name") if isinstance(obj_dict, dict) else None)
            if not class_name and isinstance(plan_dict.get("steps"), list):
                for st in plan_dict["steps"]:
                    if isinstance(st, dict) and st.get("object_id"):
                        class_name = st.get("object_id")
                        break

            scene_objects = scene_state.get("objects", [])
            matched_obj = None
            for obj in scene_objects:
                if isinstance(obj, dict):
                    if obj.get("id") == class_name or obj.get("class") == class_name or obj.get("class_name") == class_name:
                        matched_obj = obj
                        break

            if not matched_obj:
                return False, f"object_absent_for_reachability:{class_name}"

            if not matched_obj.get("reachable", False):
                reason = matched_obj.get("reason", "unreachable_in_perception")
                return False, f"object_unreachable:{reason}"

            pt = matched_obj.get("point") or matched_obj.get("position")
            if isinstance(pt, dict):
                px, py, pz = pt.get("x"), pt.get("y"), pt.get("z")
                if any(v is None or math.isnan(v) or math.isinf(v) for v in (px, py, pz)):
                    return False, "object_coordinates_invalid_nan_inf"

        return True, ""

    def check_safety_guard(self, plan_dict: Optional[Dict[str, Any]], scene_state: Optional[Dict[str, Any]]) -> Tuple[bool, str]:
        """Capa 5: Guardián de seguridad (Límites cartesianos de trabajo y estado del robot)."""
        if scene_state and isinstance(scene_state, dict):
            robot_state = scene_state.get("robot", {})
            if isinstance(robot_state, dict) and robot_state.get("busy"):
                task = plan_dict.get("task") if plan_dict else None
                if task not in ("abort", "observe_scene"):
                    return False, "robot_is_busy"

        if not plan_dict or not isinstance(plan_dict, dict):
            return False, "invalid_plan_dict"

        task = plan_dict.get("task")
        if task in ("home", "go_home", "open_gripper", "close_gripper", "observe_scene", "abort"):
            return True, ""

        # Comprobar si el objetivo cartesiano cae dentro de los límites del workspace
        if scene_state and isinstance(scene_state, dict):
            target_obj = plan_dict.get("target_object_id")
            obj_dict = plan_dict.get("object")
            class_name = target_obj or (obj_dict.get("class_name") if isinstance(obj_dict, dict) else None)
            scene_objects = scene_state.get("objects", [])
            for obj in scene_objects:
                if isinstance(obj, dict) and (obj.get("id") == class_name or obj.get("class") == class_name or obj.get("class_name") == class_name):
                    pt = obj.get("point") or obj.get("position")
                    if isinstance(pt, dict):
                        x, y, z = pt.get("x"), pt.get("y"), pt.get("z")
                        if any(v is None or math.isnan(v) or math.isinf(v) for v in (x, y, z)):
                            return False, "target_outside_workspace:nan_inf"
                        x_min, x_max = self.workspace_limits["workspace_x_min"], self.workspace_limits["workspace_x_max"]
                        y_min, y_max = self.workspace_limits["workspace_y_min"], self.workspace_limits["workspace_y_max"]
                        z_min, z_max = self.workspace_limits["workspace_z_min"], self.workspace_limits["workspace_z_max"]
                        if not (x_min <= x <= x_max and y_min <= y <= y_max and z_min <= z <= z_max):
                            return False, f"target_outside_workspace:({x:.3f},{y:.3f},{z:.3f})"

        return True, ""

    def evaluate_all(
        self,
        plan_raw: str,
        scene_state: Optional[Dict[str, Any]] = None,
    ) -> ValidationResult:
        """Ejecuta secuencialmente LAS 5 CAPAS sin cortocircuito y registra veredictos independientes."""
        parse_ok, parse_err, plan_dict = self.check_parse(plan_raw)
        schema_ok, schema_err = self.check_schema(plan_dict)
        grounding_ok, grounding_err = self.check_grounding(plan_dict, scene_state)
        reach_ok, reach_err = self.check_reachability(plan_dict, scene_state)
        guard_ok, guard_err = self.check_safety_guard(plan_dict, scene_state)

        # Determinar cuál habría sido la primera capa bloqueante
        first_blocking = "none"
        if not parse_ok:
            first_blocking = "parse"
        elif not schema_ok:
            first_blocking = "schema"
        elif not grounding_ok:
            first_blocking = "grounding"
        elif not reach_ok:
            first_blocking = "reachability"
        elif not guard_ok:
            first_blocking = "guard"

        return ValidationResult(
            parse_ok=parse_ok,
            parse_error=parse_err,
            schema_ok=schema_ok,
            schema_error=schema_err,
            grounding_ok=grounding_ok,
            grounding_error=grounding_err,
            reach_ok=reach_ok,
            reach_error=reach_err,
            guard_ok=guard_ok,
            guard_error=guard_err,
            first_blocking_layer=first_blocking,
            plan_dict=plan_dict,
        )

    def _has_nan_or_inf(self, obj: Any) -> bool:
        """Comprueba recursivamente si hay float('nan') o float('inf') en las estructuras."""
        if isinstance(obj, float):
            return math.isnan(obj) or math.isinf(obj)
        elif isinstance(obj, dict):
            return any(self._has_nan_or_inf(v) for v in obj.values())
        elif isinstance(obj, list):
            return any(self._has_nan_or_inf(v) for v in obj)
        return False
