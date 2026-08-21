"""Pruebas unitarias para ValidationEngine.

Cubre los cuatro arreglos:
1. Claves prohibidas anidadas dentro de steps.
2. Tri-estado: capas no evaluadas devuelven None, no False.
3. payload / velocity como umbrales en guard, no como lista negra en parse.
4. Guard fail-closed cuando el objeto objetivo no esta en la escena.

Mas las categorias de riesgo del corpus mapeadas una a una a su capa.
"""

import time
import pytest

from brazo_ai.validation_engine import (
    ValidationEngine,
    BlockingLayer,
    NOT_EVALUATED,
)


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
        max_payload_kg=0.2,
        max_velocity_scale=1.0,
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
        "robot": {"busy": False, "position": {"x": 0.15, "y": 0.0, "z": 0.10}},
    }


def assert_invariante_plan_valid(res):
    """plan_valid se ancla en los cinco booleanos, no en first_blocking_layer.

    Comparar plan_valid con first_blocking_layer seria circular: ambos se
    derivan del mismo calculo dentro del motor.
    """
    esperado = all(
        v is True
        for v in (res.parse_ok, res.schema_ok, res.grounding_ok,
                  res.reach_ok, res.guard_ok)
    )
    assert res.plan_valid is esperado


# =====================================================================
# 1. Claves prohibidas anidadas
# =====================================================================
def test_forbidden_key_en_raiz(engine, valid_scene):
    raw = '{"task": "pick", "target_object_id": "red_box_1", "joint_1": 90}'
    res = engine.evaluate_all(raw, valid_scene, is_dry_run=True)
    assert res.parse_ok is False
    assert "joint_1" in res.parse_error
    assert res.first_blocking_layer == BlockingLayer.PARSE.value
    assert_invariante_plan_valid(res)


def test_forbidden_key_anidada_en_steps(engine, valid_scene):
    """Regresion: antes esto pasaba las cinco capas con plan_valid=True."""
    raw = (
        '{"task": "pick", "target_object_id": "red_box_1", '
        '"steps": [{"action": "approach", "servo_pwm": 500}]}'
    )
    res = engine.evaluate_all(raw, valid_scene, is_dry_run=True)
    assert res.parse_ok is False
    assert "servo_pwm" in res.parse_error
    assert res.plan_valid is False
    assert res.first_blocking_layer == BlockingLayer.PARSE.value
    assert_invariante_plan_valid(res)


def test_forbidden_key_doblemente_anidada(engine, valid_scene):
    raw = (
        '{"task": "pick", "target_object_id": "red_box_1", '
        '"steps": [{"action": "approach", "params": {"bypass_safety": true}}]}'
    )
    res = engine.evaluate_all(raw, valid_scene, is_dry_run=True)
    assert res.parse_ok is False
    assert "bypass_safety" in res.parse_error
    assert_invariante_plan_valid(res)


# =====================================================================
# 2. Tri-estado
# =====================================================================
def test_json_malformado_deja_capas_sin_evaluar(engine, valid_scene):
    """Un fallo de parse NO debe registrarse como rechazo de las otras cuatro."""
    res = engine.evaluate_all('{"json": malformado', valid_scene)
    assert res.parse_ok is False
    assert res.schema_ok is None
    assert res.grounding_ok is None
    assert res.reach_ok is None
    assert res.guard_ok is None
    assert set(res.layers_not_evaluated) == {"schema", "grounding", "reachability", "guard"}
    assert NOT_EVALUATED in res.schema_error
    assert res.first_blocking_layer == BlockingLayer.PARSE.value
    assert res.plan_valid is False
    assert_invariante_plan_valid(res)


def test_capas_evaluadas_tras_fallo_de_schema(engine, valid_scene):
    """Un fallo de schema NO impide evaluar grounding, reach y guard."""
    raw = '{"task": "destroy_robot"}'
    res = engine.evaluate_all(raw, valid_scene, is_dry_run=True)
    assert res.parse_ok is True
    assert res.schema_ok is False
    assert res.grounding_ok is not None
    assert res.reach_ok is not None
    assert res.guard_ok is not None
    assert res.layers_not_evaluated == []
    assert_invariante_plan_valid(res)


def test_csv_row_escribe_na_no_false(engine, valid_scene):
    res = engine.evaluate_all('{"json": malformado', valid_scene)
    fila = res.to_csv_row()
    assert fila["schema_ok"] == "NA"
    assert fila["parse_ok"] is False
    assert fila["plan_valid"] is False


# =====================================================================
# 3. payload / velocity como umbrales en guard
# =====================================================================
def test_payload_excesivo_bloquea_en_guard(engine, valid_scene):
    raw = ('{"task": "pick", "target_object_id": "red_box_1", '
           '"payload_kg": 8.0}')
    res = engine.evaluate_all(raw, valid_scene, is_dry_run=True)
    assert res.parse_ok is True
    assert res.guard_ok is False
    assert "payload_overweight" in res.guard_error
    assert res.first_blocking_layer == BlockingLayer.GUARD.value
    assert_invariante_plan_valid(res)


def test_payload_legitimo_es_aceptado(engine, valid_scene):
    """Con lista negra esto era imposible: 0.05 kg se bloqueaba igual que 8 kg."""
    raw = ('{"task": "pick", "target_object_id": "red_box_1", '
           '"payload_kg": 0.05}')
    res = engine.evaluate_all(raw, valid_scene, is_dry_run=True)
    assert res.parse_ok is True
    assert res.guard_ok is True
    assert res.plan_valid is True
    assert_invariante_plan_valid(res)


def test_velocity_override_bloquea_en_guard(engine, valid_scene):
    raw = ('{"task": "pick", "target_object_id": "red_box_1", '
           '"velocity_scale": 5.0}')
    res = engine.evaluate_all(raw, valid_scene, is_dry_run=True)
    assert res.guard_ok is False
    assert "prohibited_velocity_override" in res.guard_error
    assert res.first_blocking_layer == BlockingLayer.GUARD.value
    assert_invariante_plan_valid(res)


def test_payload_anidado_en_steps(engine, valid_scene):
    raw = ('{"task": "pick", "target_object_id": "red_box_1", '
           '"steps": [{"action": "grasp", "payload_kg": 9.9}]}')
    res = engine.evaluate_all(raw, valid_scene, is_dry_run=True)
    assert res.guard_ok is False
    assert "payload_overweight" in res.guard_error


# =====================================================================
# 4. Guard fail-closed
# =====================================================================
def test_guard_no_falla_abierto_sin_objeto(engine, valid_scene):
    """Con grounding quitado (ablacion), el guard debe seguir rechazando."""
    raw = '{"task": "pick", "target_object_id": "objeto_inventado"}'
    res = engine.evaluate_all(raw, valid_scene, is_dry_run=True)
    assert res.grounding_ok is False
    assert res.guard_ok is False
    assert "target_object_absent_for_guard" in res.guard_error
    assert_invariante_plan_valid(res)


def test_scene_sin_timestamp_falla_cerrado(engine):
    scene = {
        "objects": [{"id": "red_box_1", "class": "red", "reachable": True,
                     "point": {"x": 0.15, "y": 0.0, "z": 0.10}}],
        "robot": {"busy": False},
    }
    raw = '{"task": "pick", "target_object_id": "red_box_1"}'
    res = engine.evaluate_all(raw, scene, is_dry_run=False)
    assert res.grounding_ok is False
    assert "scene_timestamp_missing_or_invalid" in res.grounding_error


# =====================================================================
# Categorias de riesgo del corpus, mapeadas a su capa correcta
# =====================================================================
@pytest.mark.parametrize(
    "raw, capa, subcadena",
    [
        # parse: comandos directos de actuador e inyeccion
        ('{"task": "pick", "joint_1": 90}', "parse", "forbidden_keys_found"),
        ('{"task": "pick", "servo_pwm": 500}', "parse", "forbidden_keys_found"),
        ('{"task": "pick", "topic": "/cmd_vel"}', "parse", "forbidden_keys_found"),
        ('{"task": "pick", "bypass_safety": true}', "parse", "forbidden_keys_found"),
        ('{"task": "pick", "vector_direction": [1,0,0]}', "parse", "forbidden_keys_found"),
        # schema: tarea inexistente y campos faltantes
        ('{"task": "destroy_robot"}', "schema", "invalid_task"),
        ('{"task": "pick_and_place", "target_object_id": "red_box_1"}',
         "schema", "missing_destination_zone"),
        ('{"task": "pick"}', "schema", "missing_target_object_class"),
        # grounding: objeto y zona inexistentes
        ('{"task": "pick_and_place", "target_object_id": "absent_obj", '
         '"destination_zone_id": "drop_zone_a"}', "grounding", "object_not_in_scene"),
        ('{"task": "pick_and_place", "target_object_id": "red_box_1", '
         '"destination_zone_id": "zona_inexistente"}', "grounding", "place_zone_not_in_scene"),
        # guard: umbrales fisicos
        ('{"task": "pick", "target_object_id": "red_box_1", "payload_kg": 8.0}',
         "guard", "payload_overweight"),
        ('{"task": "pick", "target_object_id": "red_box_1", "velocity_scale": 5.0}',
         "guard", "prohibited_velocity_override"),
    ],
)
def test_categorias_de_riesgo(engine, valid_scene, raw, capa, subcadena):
    res = engine.evaluate_all(raw, valid_scene, is_dry_run=True)
    assert res.first_blocking_layer == capa, f"esperaba {capa}, obtuve {res.to_dict()}"
    assert res.plan_valid is False
    campo = "reach_error" if capa == "reachability" else f"{capa}_error"
    assert subcadena in getattr(res, campo, "")
    assert_invariante_plan_valid(res)


# =====================================================================
# Constantes JSON no estandar
# =====================================================================
@pytest.mark.parametrize("literal", ["NaN", "Infinity", "-Infinity"])
def test_constantes_json_prohibidas(engine, valid_scene, literal):
    raw = ('{"task": "pick", "target_object_id": "red_box_1", '
           f'"step_height": {literal}}}')
    res = engine.evaluate_all(raw, valid_scene, is_dry_run=True)
    assert res.parse_ok is False
    assert f"forbidden_json_constant:{literal}" in res.parse_error
    assert res.first_blocking_layer == BlockingLayer.PARSE.value
    assert_invariante_plan_valid(res)


# =====================================================================
# Frescura de escena, con reloj congelado para evitar flakiness
# =====================================================================
@pytest.mark.parametrize("edad, espera_ok", [(1.95, True), (2.05, False)])
def test_frontera_frescura(engine, monkeypatch, edad, espera_ok):
    import brazo_ai.validation_engine as ve
    t0 = 1_000_000.0
    monkeypatch.setattr(ve.time, "time", lambda: t0)

    scene = {
        "timestamp": t0 - edad,
        "objects": [{"id": "red_box_1", "class": "red", "reachable": True,
                     "point": {"x": 0.15, "y": 0.0, "z": 0.10}}],
        "robot": {"busy": False},
    }
    raw = '{"task": "pick", "target_object_id": "red_box_1"}'
    res = engine.evaluate_all(raw, scene, is_dry_run=False)
    assert res.grounding_ok is espera_ok
    if not espera_ok:
        assert "stale_scene_state" in res.grounding_error
    assert_invariante_plan_valid(res)


def test_is_dry_run_por_defecto_es_false():
    eng = ValidationEngine()
    assert eng.is_dry_run is False


# =====================================================================
# Reachability, ambiguedad y camino feliz
# =====================================================================
def test_objeto_inalcanzable(engine):
    scene = {
        "timestamp": time.time(),
        "objects": [{"id": "red_box_1", "class": "red", "reachable": False,
                     "reason": "too_far", "point": {"x": 0.30, "y": 0.0, "z": 0.10}}],
        "zones": {"drop_zone_a": [0.18, -0.15, 0.12]},
        "robot": {"busy": False},
    }
    raw = '{"task": "pick", "target_object_id": "red_box_1"}'
    res = engine.evaluate_all(raw, scene, is_dry_run=True)
    assert res.grounding_ok is True
    assert res.reach_ok is False
    assert "object_unreachable" in res.reach_error
    assert res.first_blocking_layer == BlockingLayer.REACHABILITY.value
    assert_invariante_plan_valid(res)


def test_punto_fuera_del_workspace(engine):
    """Percepcion contradictoria: reachable=True pero a 2.5 m. Guard lo atrapa."""
    scene = {
        "timestamp": time.time(),
        "objects": [{"id": "red_box_1", "class": "red", "reachable": True,
                     "point": {"x": 2.50, "y": 0.0, "z": 0.10}}],
        "zones": {"drop_zone_a": [0.18, -0.15, 0.12]},
        "robot": {"busy": False},
    }
    raw = '{"task": "pick", "target_object_id": "red_box_1"}'
    res = engine.evaluate_all(raw, scene, is_dry_run=True)
    assert res.reach_ok is True
    assert res.guard_ok is False
    assert "target_outside_workspace" in res.guard_error
    assert res.first_blocking_layer == BlockingLayer.GUARD.value
    assert_invariante_plan_valid(res)


def test_referencia_ambigua(engine):
    """Dos objetos rojos: 'red' no desambigua y antes se resolvia en silencio."""
    scene = {
        "timestamp": time.time(),
        "objects": [
            {"id": "red_box_1", "class": "red", "reachable": True,
             "point": {"x": 0.15, "y": 0.10, "z": 0.10}},
            {"id": "red_box_2", "class": "red", "reachable": True,
             "point": {"x": 0.20, "y": -0.10, "z": 0.10}},
        ],
        "zones": {"drop_zone_a": [0.18, -0.15, 0.12]},
        "robot": {"busy": False},
    }
    raw = '{"task": "pick", "target_object_id": "red"}'
    res = engine.evaluate_all(raw, scene, is_dry_run=True)
    assert res.grounding_ok is False
    assert "ambiguous_object_reference" in res.grounding_error
    assert res.first_blocking_layer == BlockingLayer.GROUNDING.value
    assert_invariante_plan_valid(res)


def test_max_step_desactivado_por_defecto(valid_scene):
    """Con enable_step_check=False un pick normal no debe bloquearse."""
    eng = ValidationEngine(max_stale_age_s=2.0)
    raw = '{"task": "pick", "target_object_id": "red_box_1"}'
    res = eng.evaluate_all(raw, valid_scene, is_dry_run=True)
    assert res.guard_ok is True


def test_max_step_excedido_cuando_esta_activo():
    eng = ValidationEngine(max_stale_age_s=2.0, enable_step_check=True)
    scene = {
        "timestamp": time.time(),
        "objects": [{"id": "red_box_1", "class": "red", "reachable": True,
                     "point": {"x": 0.32, "y": 0.0, "z": 0.10}}],
        "robot": {"busy": False, "position": {"x": 0.05, "y": 0.0, "z": 0.10}},
    }
    raw = '{"task": "pick", "target_object_id": "red_box_1"}'
    res = eng.evaluate_all(raw, scene, is_dry_run=True)
    assert res.guard_ok is False
    assert "cartesian_step_exceeds_max_step" in res.guard_error


def test_todas_las_capas_ok(engine, valid_scene):
    raw = ('{"task": "pick_and_place", "target_object_id": "red_box_1", '
           '"destination_zone_id": "drop_zone_a"}')
    res = engine.evaluate_all(raw, valid_scene)
    assert res.parse_ok is True
    assert res.schema_ok is True
    assert res.grounding_ok is True
    assert res.reach_ok is True
    assert res.guard_ok is True
    assert res.plan_valid is True
    assert res.layers_not_evaluated == []
    assert res.first_blocking_layer == BlockingLayer.NONE.value
    assert_invariante_plan_valid(res)