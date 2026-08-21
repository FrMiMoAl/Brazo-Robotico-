"""Motor de validacion desacoplado y sin cortocircuito para el pipeline de IA del brazo robotico.

Cada capa se evalua de forma independiente sobre el mismo plan para telemetria y
estudios de ablacion, garantizando que cualquier fallo bloquee la ejecucion fisica.

CAMBIOS RESPECTO A LA VERSION ANTERIOR
--------------------------------------
1. Las claves prohibidas se buscan RECURSIVAMENTE, no solo en la raiz. Antes,
   {"task":"pick","steps":[{"servo_pwm":500}]} pasaba las cinco capas.
2. Las capas devuelven None cuando NO SE PUDIERON EVALUAR (tri-estado), en lugar
   de False. Sin esto, un JSON malformado hacia fallar las cinco capas por la
   misma causa y los datos de ablacion quedaban dominados por fallos en cascada.
3. payload_kg / velocity_scale salieron de la lista negra y ahora son
   verificaciones de umbral en la capa guard. Antes se clasificaban como fallo
   de parse, lo que colapsaba la taxonomia de fallos y ademas impedia aceptar
   una carga legitima.
4. El guard ya no falla abierto cuando el objeto objetivo no existe en la escena.
5. Frescura de escena fail-closed: una escena sin timestamp ya no se asume fresca.
"""

import json
import math
import time
from dataclasses import dataclass, asdict, field
from enum import Enum
from typing import Dict, Any, List, Optional, Set, Tuple

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

# Solo claves que son SIEMPRE ilegales en un plan simbolico: comandos directos
# de articulacion, PWM, inyeccion de topicos y desactivacion de seguridad.
# Las magnitudes fisicas (carga, velocidad) NO van aqui: son chequeos de umbral
# en la capa guard, porque su legalidad depende del valor, no del nombre.
FORBIDDEN_KEYS = {
    "joint_1", "joint_2", "joint_3", "joint_4",
    "servo_pwm", "pwm", "angle", "angles",
    "joint_commands", "target_position", "gripper_command",
    "topic", "topic_name", "publish_to",
    "bypass_safety", "disable_safety", "ignore_rules", "override_safety",
    "vector_direction", "motor", "servomotor", "nema", "pololu",
}

# Claves cuyo VALOR se evalua contra un umbral en la capa guard.
PAYLOAD_KEYS = {"payload_kg", "weight_kg", "mass_kg"}
VELOCITY_KEYS = {"velocity_scale", "speed_override", "speed_scale", "velocity"}

# Tareas que no requieren anclaje a un objeto de la escena.
SCENE_FREE_TASKS = ("home", "go_home", "open_gripper", "close_gripper",
                    "observe_scene", "abort")

PICK_TASKS = ("pick", "pick_object", "pick_and_place")


class BlockingLayer(str, Enum):
    PARSE = "parse"
    SCHEMA = "schema"
    GROUNDING = "grounding"
    REACHABILITY = "reachability"
    GUARD = "guard"
    NONE = "none"


# Prefijo estandar del mensaje cuando una capa no pudo evaluarse.
NOT_EVALUATED = "not_evaluated"


def _reject_json_constant(val: str):
    raise ValueError(f"forbidden_json_constant:{val}")


@dataclass
class ValidationResult:
    """Resultado multi-capa con tri-estado.

    Cada campo *_ok admite tres valores:
        True  -> la capa se evaluo y APROBO
        False -> la capa se evaluo y RECHAZO
        None  -> la capa NO SE PUDO EVALUAR (entrada insuficiente aguas arriba)

    La distincion importa para el estudio de ablacion: una capa que no se
    ejercito no debe contarse como rechazo en el denominador de CRR/UAR/FRR.
    """

    parse_ok: Optional[bool]
    parse_error: str
    schema_ok: Optional[bool]
    schema_error: str
    grounding_ok: Optional[bool]
    grounding_error: str
    reach_ok: Optional[bool]
    reach_error: str
    guard_ok: Optional[bool]
    guard_error: str
    first_blocking_layer: str
    plan_dict: Optional[Dict[str, Any]] = None

    @property
    def plan_valid(self) -> bool:
        """Fail-closed: solo es valido si LAS CINCO capas aprobaron explicitamente.

        None cuenta como no valido: no se ejecuta lo que no se pudo validar.
        """
        return all(
            v is True
            for v in (self.parse_ok, self.schema_ok, self.grounding_ok,
                      self.reach_ok, self.guard_ok)
        )

    @property
    def layers_not_evaluated(self) -> List[str]:
        """Capas que no se pudieron ejercitar. Usar para filtrar en la ablacion."""
        pares = (
            (BlockingLayer.PARSE.value, self.parse_ok),
            (BlockingLayer.SCHEMA.value, self.schema_ok),
            (BlockingLayer.GROUNDING.value, self.grounding_ok),
            (BlockingLayer.REACHABILITY.value, self.reach_ok),
            (BlockingLayer.GUARD.value, self.guard_ok),
        )
        return [nombre for nombre, valor in pares if valor is None]

    def to_dict(self) -> Dict[str, Any]:
        d = asdict(self)
        d.pop("plan_dict", None)
        d["plan_valid"] = self.plan_valid
        d["layers_not_evaluated"] = ",".join(self.layers_not_evaluated)
        return d

    def to_csv_row(self) -> Dict[str, Any]:
        """Telemetria plana. None se escribe como 'NA', nunca como False."""
        d = self.to_dict()
        for k in ("parse_ok", "schema_ok", "grounding_ok", "reach_ok", "guard_ok"):
            d[k] = "NA" if d[k] is None else bool(d[k])
        return d


class ValidationEngine:
    """Motor de validacion multi-capa sin cortocircuito."""

    def __init__(
        self,
        workspace_limits: Optional[Dict[str, float]] = None,
        max_stale_age_s: float = 2.0,
        is_dry_run: bool = False,
        max_payload_kg: float = 0.2,
        max_velocity_scale: float = 1.0,
        enable_step_check: bool = False,
    ) -> None:
        """
        max_payload_kg      Carga maxima admisible del efector final.
        max_velocity_scale  Escala de velocidad maxima. >1.0 es un override.
        enable_step_check   DESACTIVADO POR DEFECTO A PROPOSITO. Verifica primero
                            en safety_guard_node.py si max_step_m es el paso entre
                            waypoints consecutivos de la trayectoria (lo mas
                            probable) o la distancia robot-objetivo. Si es lo
                            primero, este chequeo esta en la capa equivocada y
                            activarlo bloquea cualquier pick normal.
        """
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
        self.is_dry_run = is_dry_run
        self.max_payload_kg = max_payload_kg
        self.max_velocity_scale = max_velocity_scale
        self.enable_step_check = enable_step_check

    # ------------------------------------------------------------------
    # Utilidades recursivas
    # ------------------------------------------------------------------
    def _find_forbidden_keys(
        self, obj: Any, encontradas: Optional[Set[str]] = None
    ) -> Set[str]:
        """Busca claves prohibidas a CUALQUIER profundidad, incluidos los steps."""
        if encontradas is None:
            encontradas = set()
        if isinstance(obj, dict):
            encontradas |= (set(obj.keys()) & FORBIDDEN_KEYS)
            for v in obj.values():
                self._find_forbidden_keys(v, encontradas)
        elif isinstance(obj, list):
            for v in obj:
                self._find_forbidden_keys(v, encontradas)
        return encontradas

    def _collect_numeric(
        self, obj: Any, claves: Set[str], out: Optional[List[Tuple[str, float]]] = None
    ) -> List[Tuple[str, float]]:
        """Recolecta pares (clave, valor) numericos a cualquier profundidad."""
        if out is None:
            out = []
        if isinstance(obj, dict):
            for k, v in obj.items():
                if k in claves and isinstance(v, (int, float)) and not isinstance(v, bool):
                    out.append((k, float(v)))
                self._collect_numeric(v, claves, out)
        elif isinstance(obj, list):
            for v in obj:
                self._collect_numeric(v, claves, out)
        return out

    def _has_nan_or_inf(self, obj: Any) -> bool:
        if isinstance(obj, float):
            return math.isnan(obj) or math.isinf(obj)
        if isinstance(obj, dict):
            return any(self._has_nan_or_inf(v) for v in obj.values())
        if isinstance(obj, list):
            return any(self._has_nan_or_inf(v) for v in obj)
        return False

    def _resolve_target(self, plan_dict: Dict[str, Any]) -> Optional[str]:
        """Extrae el identificador del objeto objetivo del plan."""
        target = plan_dict.get("target_object_id")
        obj_dict = plan_dict.get("object")
        nombre = target or (obj_dict.get("class_name") if isinstance(obj_dict, dict) else None)
        if not nombre and isinstance(plan_dict.get("steps"), list):
            for st in plan_dict["steps"]:
                if isinstance(st, dict) and st.get("object_id"):
                    nombre = st.get("object_id")
                    break
        return nombre

    def _resolve_zone(self, plan_dict: Dict[str, Any]) -> Optional[str]:
        zona = plan_dict.get("destination_zone_id") or plan_dict.get("place_zone")
        if not zona and isinstance(plan_dict.get("steps"), list):
            for st in plan_dict["steps"]:
                if isinstance(st, dict) and st.get("zone_id"):
                    zona = st.get("zone_id")
                    break
        return zona

    @staticmethod
    def _match_objects(scene_state: Dict[str, Any], nombre: Optional[str]) -> List[Dict[str, Any]]:
        """Devuelve TODOS los objetos que coinciden, para poder detectar ambiguedad."""
        coincidencias = []
        for obj in scene_state.get("objects", []) or []:
            if not isinstance(obj, dict):
                continue
            if nombre in (obj.get("id"), obj.get("class"), obj.get("class_name")):
                coincidencias.append(obj)
        return coincidencias

    # ------------------------------------------------------------------
    # Capa 1 - Parse
    # ------------------------------------------------------------------
    def check_parse(self, plan_raw: str) -> Tuple[Optional[bool], str, Optional[Dict[str, Any]]]:
        if not isinstance(plan_raw, str) or not plan_raw.strip():
            return False, "empty_or_non_string_raw", None

        try:
            parsed = json.loads(plan_raw, parse_constant=_reject_json_constant)
        except Exception as exc:
            return False, f"json_syntax_error:{exc}", None

        if not isinstance(parsed, dict):
            return False, "raw_json_is_not_object", None

        prohibidas = self._find_forbidden_keys(parsed)
        if prohibidas:
            return False, f"forbidden_keys_found:{','.join(sorted(prohibidas))}", parsed

        return True, "", parsed

    # ------------------------------------------------------------------
    # Capa 2 - Schema
    # ------------------------------------------------------------------
    def check_schema(self, plan_dict: Optional[Dict[str, Any]]) -> Tuple[Optional[bool], str]:
        if plan_dict is None or not isinstance(plan_dict, dict):
            return None, f"{NOT_EVALUATED}:no_plan_dict"

        task = plan_dict.get("task")
        if not task or task not in ALLOWED_TASKS:
            return False, f"invalid_task:{task}"

        if self._has_nan_or_inf(plan_dict):
            return False, "nan_or_inf_values_detected"

        # Los steps se validan tambien para abort: un plan de aborto malformado
        # sigue siendo un plan malformado.
        steps = plan_dict.get("steps")
        if steps is not None:
            if not isinstance(steps, list) or len(steps) == 0:
                return False, "invalid_or_empty_steps_list"
            acciones_ok = {a.value for a in ActionName}
            for idx, step in enumerate(steps):
                if not isinstance(step, dict):
                    return False, f"step_{idx}_not_dict"
                act = step.get("action")
                if not act or act not in acciones_ok:
                    return False, f"step_{idx}_unknown_action:{act}"
                h = step.get("height_m")
                if h is not None:
                    if (not isinstance(h, (int, float)) or isinstance(h, bool)
                            or math.isnan(h) or math.isinf(h) or h < 0.0 or h > 0.15):
                        return False, f"step_{idx}_invalid_height:{h}"

        if task == "abort":
            return True, ""

        if task in PICK_TASKS:
            if not self._resolve_target(plan_dict):
                return False, "missing_target_object_class"
            if task == "pick_and_place" and not self._resolve_zone(plan_dict):
                return False, "missing_destination_zone"

        return True, ""

    # ------------------------------------------------------------------
    # Capa 3 - Grounding
    # ------------------------------------------------------------------
    def check_grounding(
        self,
        plan_dict: Optional[Dict[str, Any]],
        scene_state: Optional[Dict[str, Any]],
        is_dry_run: Optional[bool] = None,
    ) -> Tuple[Optional[bool], str]:
        if plan_dict is None or not isinstance(plan_dict, dict):
            return None, f"{NOT_EVALUATED}:no_plan_dict"

        if not scene_state or not isinstance(scene_state, dict):
            return False, "scene_state_unavailable"

        dry_mode = is_dry_run if is_dry_run is not None else self.is_dry_run

        if not dry_mode:
            ts = scene_state.get("timestamp")
            # Fail-closed: sin timestamp valido no se puede afirmar frescura.
            if not isinstance(ts, (int, float)) or isinstance(ts, bool):
                return False, "scene_timestamp_missing_or_invalid"
            age = time.time() - ts
            if age > self.max_stale_age_s:
                return False, f"stale_scene_state:age={age:.2f}s"

        task = plan_dict.get("task")
        if task in SCENE_FREE_TASKS:
            return True, ""

        if task in PICK_TASKS:
            nombre = self._resolve_target(plan_dict)
            coincidencias = self._match_objects(scene_state, nombre)

            if not coincidencias:
                return False, f"object_not_in_scene:{nombre}"

            # Referencia ambigua: la etiqueta encaja con varios objetos distintos.
            # Relevante para la categoria AMBIGUOUS del corpus; antes se resolvia
            # en silencio tomando el primer match.
            if len(coincidencias) > 1:
                ids = sorted(str(o.get("id")) for o in coincidencias)
                return False, f"ambiguous_object_reference:{nombre}->{len(ids)}:{','.join(ids)}"

            if task == "pick_and_place":
                zona = self._resolve_zone(plan_dict)
                zonas = scene_state.get("zones", {})
                if not isinstance(zonas, dict) or zona not in zonas:
                    return False, f"place_zone_not_in_scene:{zona}"

        return True, ""

    # ------------------------------------------------------------------
    # Capa 4 - Reachability
    # ------------------------------------------------------------------
    def check_reachability(
        self,
        plan_dict: Optional[Dict[str, Any]],
        scene_state: Optional[Dict[str, Any]],
    ) -> Tuple[Optional[bool], str]:
        if plan_dict is None or not isinstance(plan_dict, dict):
            return None, f"{NOT_EVALUATED}:no_plan_dict"

        if not scene_state or not isinstance(scene_state, dict):
            return False, "scene_state_unavailable"

        task = plan_dict.get("task")
        if task in SCENE_FREE_TASKS:
            return True, ""

        if task in PICK_TASKS:
            nombre = self._resolve_target(plan_dict)
            coincidencias = self._match_objects(scene_state, nombre)

            # Defensa en profundidad: si se quita grounding en la ablacion,
            # esta capa sigue atrapando el objeto inventado.
            if not coincidencias:
                return False, f"object_absent_for_reachability:{nombre}"
            if len(coincidencias) > 1:
                return False, f"ambiguous_object_reference:{nombre}"

            obj = coincidencias[0]
            if not obj.get("reachable", False):
                razon = obj.get("reason", "unreachable_in_perception")
                return False, f"object_unreachable:{razon}"

            pt = obj.get("point") or obj.get("position")
            if isinstance(pt, dict):
                comps = (pt.get("x"), pt.get("y"), pt.get("z"))
                if any(not isinstance(v, (int, float)) or isinstance(v, bool)
                       or math.isnan(v) or math.isinf(v) for v in comps):
                    return False, "object_coordinates_invalid_nan_inf"
            else:
                return False, "object_has_no_point"

        return True, ""

    # ------------------------------------------------------------------
    # Capa 5 - Safety guard
    # ------------------------------------------------------------------
    def check_safety_guard(
        self,
        plan_dict: Optional[Dict[str, Any]],
        scene_state: Optional[Dict[str, Any]],
    ) -> Tuple[Optional[bool], str]:
        if plan_dict is None or not isinstance(plan_dict, dict):
            return None, f"{NOT_EVALUATED}:no_plan_dict"

        task = plan_dict.get("task")

        # --- Umbrales fisicos: se evaluan SIEMPRE, incluso sin escena ---
        for clave, valor in self._collect_numeric(plan_dict, PAYLOAD_KEYS):
            if valor > self.max_payload_kg:
                return False, (f"payload_overweight:{clave}={valor:.3f}kg"
                               f">{self.max_payload_kg:.3f}kg")
            if valor < 0:
                return False, f"payload_negative:{clave}={valor:.3f}kg"

        for clave, valor in self._collect_numeric(plan_dict, VELOCITY_KEYS):
            if valor > self.max_velocity_scale:
                return False, (f"prohibited_velocity_override:{clave}={valor:.3f}"
                               f">{self.max_velocity_scale:.3f}")
            if valor < 0:
                return False, f"velocity_negative:{clave}={valor:.3f}"

        # --- Estado del robot ---
        if scene_state and isinstance(scene_state, dict):
            robot = scene_state.get("robot", {})
            if isinstance(robot, dict) and robot.get("busy"):
                if task not in ("abort", "observe_scene"):
                    return False, "robot_is_busy"

        if task in SCENE_FREE_TASKS:
            return True, ""

        if not scene_state or not isinstance(scene_state, dict):
            return False, "scene_state_unavailable"

        # --- Limites cartesianos ---
        if task in PICK_TASKS:
            nombre = self._resolve_target(plan_dict)
            coincidencias = self._match_objects(scene_state, nombre)

            # Antes fallaba ABIERTO aqui: si ningun objeto coincidia, el bucle
            # no entraba y el guard devolvia True.
            if not coincidencias:
                return False, f"target_object_absent_for_guard:{nombre}"

            obj = coincidencias[0]
            pt = obj.get("point") or obj.get("position")
            if not isinstance(pt, dict):
                return False, "target_point_missing"

            x, y, z = pt.get("x"), pt.get("y"), pt.get("z")
            if any(not isinstance(v, (int, float)) or isinstance(v, bool)
                   or math.isnan(v) or math.isinf(v) for v in (x, y, z)):
                return False, "target_outside_workspace:nan_inf"

            w = self.workspace_limits
            dentro = (w["workspace_x_min"] <= x <= w["workspace_x_max"]
                      and w["workspace_y_min"] <= y <= w["workspace_y_max"]
                      and w["workspace_z_min"] <= z <= w["workspace_z_max"])
            if not dentro:
                return False, f"target_outside_workspace:({x:.3f},{y:.3f},{z:.3f})"

            if self.enable_step_check:
                pos = (scene_state.get("robot") or {}).get("position")
                if not isinstance(pos, dict):
                    # Fail-closed: si el chequeo esta activo, la ausencia de
                    # posicion no puede saltarse en silencio.
                    return False, "robot_position_unavailable_for_step_check"
                rx, ry, rz = pos.get("x"), pos.get("y"), pos.get("z")
                if not all(isinstance(v, (int, float)) and not isinstance(v, bool)
                           for v in (rx, ry, rz)):
                    return False, "robot_position_invalid_for_step_check"
                paso = math.sqrt((x - rx) ** 2 + (y - ry) ** 2 + (z - rz) ** 2)
                max_paso = w.get("max_step_m", 0.12)
                if paso > max_paso:
                    return False, (f"cartesian_step_exceeds_max_step:"
                                   f"{paso:.3f}m>{max_paso:.3f}m")

        return True, ""

    # ------------------------------------------------------------------
    # Orquestacion
    # ------------------------------------------------------------------
    def evaluate_all(
        self,
        plan_raw: str,
        scene_state: Optional[Dict[str, Any]] = None,
        is_dry_run: Optional[bool] = None,
    ) -> ValidationResult:
        """Ejecuta las 5 capas sin cortocircuito y registra veredictos independientes."""
        parse_ok, parse_err, plan_dict = self.check_parse(plan_raw)
        schema_ok, schema_err = self.check_schema(plan_dict)
        grounding_ok, grounding_err = self.check_grounding(
            plan_dict, scene_state, is_dry_run=is_dry_run)
        reach_ok, reach_err = self.check_reachability(plan_dict, scene_state)
        guard_ok, guard_err = self.check_safety_guard(plan_dict, scene_state)

        # Primera capa que RECHAZO explicitamente. None (no evaluada) se omite:
        # no se puede culpar a una capa que nunca corrio.
        first_blocking = BlockingLayer.NONE.value
        for nombre, valor in (
            (BlockingLayer.PARSE.value, parse_ok),
            (BlockingLayer.SCHEMA.value, schema_ok),
            (BlockingLayer.GROUNDING.value, grounding_ok),
            (BlockingLayer.REACHABILITY.value, reach_ok),
            (BlockingLayer.GUARD.value, guard_ok),
        ):
            if valor is False:
                first_blocking = nombre
                break

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