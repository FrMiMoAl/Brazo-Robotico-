"""Utilidades compartidas para construir y validar planes JSON de alto nivel.

Usado por llm_agent_node.py y manual_plan_node.py. El LLM (o el modo
determinista sin LLM) SOLO debe producir planes de este tipo: nunca
angulos, PWM ni topicos de control directo.
"""

import json

ALLOWED_TASKS = [
    "observe_scene",
    "pick_object",
    "pick_and_place",
    "open_gripper",
    "close_gripper",
    "go_home",
    "abort",
]

# Si el plan generado por una API externa contiene cualquiera de estas
# claves, se rechaza: serian señal de que el LLM intento controlar
# motores directamente, lo cual esta prohibido por diseño.
FORBIDDEN_KEYS = [
    "joint_1", "joint_2", "joint_3", "joint_4",
    "servo_pwm", "pwm", "angle", "angles", "velocity", "velocities",
    "joint_commands", "target_position", "gripper_command",
]

# Mapeo simple texto -> clase de objeto. MVP solo soporta "red" (deteccion
# HSV). Ampliar aqui cuando haya mas clases soportadas por el detector.
CLASS_KEYWORDS = {
    "red": "red",
    "rojo": "red",
}

PICK_KEYWORDS = ["agarra", "agarrar", "recoge", "recoger", "toma", "pick", "grab"]
OPEN_GRIPPER_KEYWORDS = ["abre", "abrir", "suelta", "soltar", "open"]
CLOSE_GRIPPER_KEYWORDS = ["cierra", "cerrar", "close"]
HOME_KEYWORDS = ["home", "inicio", "casa"]
OBSERVE_KEYWORDS = ["observa", "mira", "escanea", "observe", "look", "scan"]
ABORT_KEYWORDS = ["detente", "para", "abort", "stop", "cancela", "cancelar"]


def abort_plan(reason: str) -> dict:
    return {"task": "abort", "reason": reason}


def build_pick_and_place_plan(class_name: str, place_zone: str, selection: str = "largest_reachable") -> dict:
    return {
        "task": "pick_and_place",
        "object": {
            "class_name": class_name,
            "selection": selection,
        },
        "place_zone": place_zone,
    }


def find_class_in_text(text: str):
    text_l = text.lower()
    for keyword, class_name in CLASS_KEYWORDS.items():
        if keyword in text_l:
            return class_name
    return None


def parse_user_command(text: str, default_place_zone: str) -> dict:
    """Modo determinista sin LLM: traduce texto humano simple a un plan JSON.

    Reglas: solo produce 'pick_and_place', 'open_gripper', 'close_gripper',
    'go_home', 'observe_scene' o 'abort'. Nunca inventa coordenadas.
    """
    text_l = text.lower().strip()

    if any(k in text_l for k in ABORT_KEYWORDS):
        return abort_plan("user_requested_abort")

    if any(k in text_l for k in PICK_KEYWORDS):
        class_name = find_class_in_text(text_l)
        if class_name is None:
            return abort_plan("unknown_object_class")
        return build_pick_and_place_plan(class_name, default_place_zone)

    if any(k in text_l for k in OPEN_GRIPPER_KEYWORDS):
        return {"task": "open_gripper"}

    if any(k in text_l for k in CLOSE_GRIPPER_KEYWORDS):
        return {"task": "close_gripper"}

    if any(k in text_l for k in HOME_KEYWORDS):
        return {"task": "go_home"}

    if any(k in text_l for k in OBSERVE_KEYWORDS):
        return {"task": "observe_scene"}

    return abort_plan("command_not_understood")


def find_reachable_object(scene_state: dict, class_name: str):
    objects = scene_state.get("objects", []) if scene_state else []
    candidates = [
        o for o in objects
        if o.get("class") == class_name and o.get("reachable") is True
    ]
    if not candidates:
        return None
    candidates.sort(key=lambda o: o.get("confidence", 0.0), reverse=True)
    return candidates[0]


def validate_plan(plan: dict, scene_state: dict):
    """Valida un plan antes de publicarlo en /llm_plan.

    Devuelve (valid, reason). No modifica el plan: si no es valido, quien
    llama debe sustituirlo por abort_plan(reason).
    """
    if not isinstance(plan, dict):
        return False, "plan_not_json_object"

    for key in plan.keys():
        if key in FORBIDDEN_KEYS:
            return False, f"forbidden_key:{key}"

    task = plan.get("task")
    if task not in ALLOWED_TASKS:
        return False, "task_not_allowed"

    if task == "abort":
        return True, "ok"

    if task in ("pick_object", "pick_and_place"):
        obj = plan.get("object", {})
        if not isinstance(obj, dict) or "class_name" not in obj:
            return False, "missing_object_class_name"

        class_name = obj["class_name"]
        scene_classes = {o.get("class") for o in scene_state.get("objects", [])} if scene_state else set()
        if class_name not in scene_classes:
            return False, "object_class_not_in_scene"

        target = find_reachable_object(scene_state, class_name)
        if target is None:
            return False, "no_reachable_object"

        if task == "pick_and_place":
            place_zone = plan.get("place_zone")
            zones = scene_state.get("zones", {}) if scene_state else {}
            if place_zone not in zones:
                return False, "place_zone_not_in_scene"

    if scene_state and scene_state.get("robot", {}).get("busy"):
        if task not in ("abort", "observe_scene"):
            return False, "robot_busy"

    return True, "ok"


def plan_to_json(plan: dict) -> str:
    return json.dumps(plan)
