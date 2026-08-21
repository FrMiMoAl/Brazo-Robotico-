"""Pruebas de integración sobre LlmAgentNode (Orquestador de Inferencia y Validación).

Prueba que el snapshot de escena utilizado para la validación de grounding
en el sistema real (dry_run=False) se re-consulta de forma fresca tras el retorno del LLM,
y que en dry_run=True se exime de la frescura congelando el snapshot.
"""

import json
import time
from unittest.mock import MagicMock

import pytest
import rclpy
from brazo_ai.llm_agent_node import LlmAgentNode


@pytest.fixture(scope="module")
def rclpy_init():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def test_scene_is_refreshed_after_inference(rclpy_init):
    """En el sistema real (dry_run=False), la validación de grounding DEBE usar

    el estado de escena re-consultado tras la inferencia del LLM.
    """
    node = LlmAgentNode()
    node.dry_run = False
    node.validation_engine.is_dry_run = False

    # Escena inicial vieja
    t_old = time.time() - 5.0
    initial_scene = {
        "timestamp": t_old,
        "objects": [
            {
                "id": "red_box_1",
                "class": "red",
                "reachable": True,
                "point": {"x": 0.15, "y": 0.0, "z": 0.10},
            }
        ],
        "robot": {"busy": False, "position": {"x": 0.15, "y": 0.0, "z": 0.10}},
    }
    node.current_scene = initial_scene

    # Mock del backend LLM que simula tiempo de inferencia y actualiza current_scene en medio
    def mock_generate_plan(system_prompt, user_prompt):
        time.sleep(0.05)
        # Actualización de escena que ocurre durante/al final de la inferencia
        now_ts = time.time()
        node.current_scene = {
            "timestamp": now_ts,
            "objects": [
                {
                    "id": "red_box_1",
                    "class": "red",
                    "reachable": True,
                    "point": {"x": 0.15, "y": 0.0, "z": 0.10},
                }
            ],
            "robot": {"busy": False, "position": {"x": 0.15, "y": 0.0, "z": 0.10}},
        }
        res = MagicMock()
        res.raw_response = '{"task": "pick", "target_object_id": "red_box_1"}'
        res.latency_s = 0.05
        res.input_tokens = 15
        res.output_tokens = 25
        return res

    mock_backend = MagicMock()
    mock_backend.generate_plan.side_effect = mock_generate_plan
    node.backend = mock_backend
    node.use_llm_api = True

    published_plans = []
    node.plan_publisher = MagicMock()
    node.plan_publisher.publish.side_effect = lambda msg: published_plans.append(json.loads(msg.data))

    node._process_command_with_llm("agarra la caja roja")

    assert len(published_plans) == 1
    plan_msg = published_plans[0]
    telemetry = plan_msg["telemetry"]

    # Al re-consultar la escena tras la inferencia, grounding_ok es True
    assert telemetry["grounding_ok"] is True
    assert telemetry["plan_valid"] is True
    assert telemetry["first_blocking_layer"] == "none"

    node.destroy_node()


def test_dry_run_scene_frozen_snapshot(rclpy_init):
    """En dry_run=True, el snapshot inicial se congela y exime el chequeo de frescura."""
    node = LlmAgentNode()
    node.dry_run = True
    node.validation_engine.is_dry_run = True

    t_old = time.time() - 10.0
    stale_scene = {
        "timestamp": t_old,
        "objects": [
            {
                "id": "red_box_1",
                "class": "red",
                "reachable": True,
                "point": {"x": 0.15, "y": 0.0, "z": 0.10},
            }
        ],
        "robot": {"busy": False, "position": {"x": 0.15, "y": 0.0, "z": 0.10}},
    }
    node.current_scene = stale_scene

    mock_backend = MagicMock()
    mock_res = MagicMock()
    mock_res.raw_response = '{"task": "pick", "target_object_id": "red_box_1"}'
    mock_res.latency_s = 0.01
    mock_res.input_tokens = 12
    mock_res.output_tokens = 18
    mock_backend.generate_plan.return_value = mock_res
    node.backend = mock_backend
    node.use_llm_api = True

    published_plans = []
    node.plan_publisher = MagicMock()
    node.plan_publisher.publish.side_effect = lambda msg: published_plans.append(json.loads(msg.data))

    node._process_command_with_llm("agarra la caja roja")

    assert len(published_plans) == 1
    plan_msg = published_plans[0]
    telemetry = plan_msg["telemetry"]

    assert telemetry["grounding_ok"] is True
    assert telemetry["plan_valid"] is True

    node.destroy_node()
