"""Pruebas unitarias para ValidationEngine.

Cubre: JSON malformado, claves de motor prohibidas, accion desconocida,
objeto inventado, objeto ausente/inalcanzable, punto fuera de workspace,
valores NaN/Infinito y escena caducada.
"""

import time
import pytest
from brazo_ai.validation_engine import ValidationEngine


@pytest.fixture
def engine():
    return ValidationEngine(
        workspace_limits={
            "workspace_x_min": 0.05,
            "workspace_x_max": 0.35,
            "workspace_y_min": -0.25,
            "workspace_y_max": 0.25,
            "workspace_z_min": 0.03,
            "workspace_z_max": 0.35,
            "max_step_m": 0.12,
        },
        max_stale_age_s=2.0,
    )


@pytest.fixture
def valid_scene():
    return {
        "timestamp": time.time(),
        "objects": [
            {
                "id": "red_box_1",
                "class": "red",
                "confidence": 0.95,
                "reachable": True,
                "reason": "in_workspace",
                "point": {"x": 0.15, "y": 0.0, "z": 0.10},
            }
        ],
        "zones": {
            "drop_zone_a": [0.18, -0.15, 0.12],
            "drop_zone_b": [0.18, 0.15, 0.12],
        },
        "robot": {"busy": False},
    }


def test_malformed_json(engine, valid_scene):
    res = engine.evaluate_all('{"json": malformado', valid_scene)
    assert res.parse_ok is False
    assert "json_syntax_error" in res.parse_error
    assert res.first_blocking_layer == "parse"


def test_forbidden_motor_keys(engine, valid_scene):
    raw = '{"task": "pick", "joint_1": 90, "servo_pwm": 500}'
    res = engine.evaluate_all(raw, valid_scene)
    assert res.parse_ok is False
    assert "forbidden_keys_found" in res.parse_error
    assert res.first_blocking_layer == "parse"


def test_unknown_action_task(engine, valid_scene):
    raw = '{"task": "fly_to_moon", "target_object_id": "red_box_1"}'
    res = engine.evaluate_all(raw, valid_scene)
    assert res.parse_ok is True
    assert res.schema_ok is False
    assert "invalid_task" in res.schema_error
    assert res.first_blocking_layer == "schema"


def test_invented_object(engine, valid_scene):
    raw = '{"task": "pick_and_place", "target_object_id": "blue_ball", "destination_zone_id": "drop_zone_a"}'
    res = engine.evaluate_all(raw, valid_scene)
    assert res.parse_ok is True
    assert res.schema_ok is True
    assert res.grounding_ok is False
    assert "object_not_in_scene" in res.grounding_error
    assert res.first_blocking_layer == "grounding"


def test_unreachable_object(engine):
    scene = {
        "timestamp": time.time(),
        "objects": [
            {
                "id": "red_box_1",
                "class": "red",
                "reachable": False,
                "reason": "too_far",
                "point": {"x": 0.80, "y": 0.0, "z": 0.10},
            }
        ],
        "zones": {"drop_zone_a": [0.18, -0.15, 0.12]},
    }
    raw = '{"task": "pick", "target_object_id": "red_box_1"}'
    res = engine.evaluate_all(raw, scene)
    assert res.parse_ok is True
    assert res.schema_ok is True
    assert res.grounding_ok is True
    assert res.reach_ok is False
    assert "object_unreachable" in res.reach_error
    assert res.first_blocking_layer == "reachability"


def test_point_outside_workspace(engine):
    scene = {
        "timestamp": time.time(),
        "objects": [
            {
                "id": "red_box_1",
                "class": "red",
                "reachable": True,
                "point": {"x": 2.50, "y": 0.0, "z": 0.10},
            }
        ],
        "zones": {"drop_zone_a": [0.18, -0.15, 0.12]},
    }
    raw = '{"task": "pick", "target_object_id": "red_box_1"}'
    res = engine.evaluate_all(raw, scene)
    assert res.parse_ok is True
    assert res.schema_ok is True
    assert res.grounding_ok is True
    assert res.reach_ok is True
    assert res.guard_ok is False
    assert "target_outside_workspace" in res.guard_error
    assert res.first_blocking_layer == "guard"


def test_nan_infinity_values(engine, valid_scene):
    raw = '{"task": "pick", "target_object_id": "red_box_1", "steps": [{"action": "approach", "height_m": NaN}]}'
    res = engine.evaluate_all(raw, valid_scene)
    assert res.parse_ok is False or res.schema_ok is False


def test_stale_scene_state(engine):
    stale_scene = {
        "timestamp": time.time() - 10.0,  # 10s old
        "objects": [
            {
                "id": "red_box_1",
                "class": "red",
                "reachable": True,
                "point": {"x": 0.15, "y": 0.0, "z": 0.10},
            }
        ],
    }
    raw = '{"task": "pick", "target_object_id": "red_box_1"}'
    res = engine.evaluate_all(raw, stale_scene)
    assert res.grounding_ok is False
    assert "stale_scene_state" in res.grounding_error


def test_all_layers_ok(engine, valid_scene):
    raw = '{"task": "pick_and_place", "target_object_id": "red_box_1", "destination_zone_id": "drop_zone_a"}'
    res = engine.evaluate_all(raw, valid_scene)
    assert res.parse_ok is True
    assert res.schema_ok is True
    assert res.grounding_ok is True
    assert res.reach_ok is True
    assert res.guard_ok is True
    assert res.first_blocking_layer == "none"
