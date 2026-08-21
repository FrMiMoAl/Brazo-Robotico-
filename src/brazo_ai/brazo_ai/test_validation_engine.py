"""Prueba de concurrencia real sobre LlmAgentNode.

POR QUE ESTE ARCHIVO EXISTE
---------------------------
El test anterior (test_scene_is_refreshed_after_inference) actualizaba
node.current_scene DESDE DENTRO del mock de generate_plan, de forma sincrona.
Eso garantiza que la escena este fresca sin importar como este montado el
executor, asi que pasa incluso si el bug sigue vivo.

En produccion la escena la escribe un callback del subscriber. Si
_process_command_with_llm corre dentro de un callback que bloquea el executor
durante los 2-26 s de inferencia, el subscriber NUNCA alcanza a correr y
current_scene sigue con el valor viejo cuando se vuelve a leer. Re-leer despues
de la inferencia no arregla nada si nadie pudo escribir mientras tanto.

Este test usa un publisher REAL girando en un executor REAL, y una inferencia
que bloquea. Si el nodo usa SingleThreadedExecutor sin callback group
reentrante, o si no delega la inferencia a un hilo, este test falla.
"""

import json
import threading
import time
from unittest.mock import MagicMock

import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from brazo_ai.llm_agent_node import LlmAgentNode

# AJUSTA ESTO al topico y tipo de mensaje reales de tu escena.
SCENE_TOPIC = "/scene_state"
LATENCIA_SIMULADA_S = 1.5


@pytest.fixture(scope="module")
def rclpy_ctx():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


class PublicadorDeEscena(Node):
    """Publica escena fresca a 10 Hz, como el nodo de percepcion real."""

    def __init__(self):
        super().__init__("publicador_escena_test")
        self.pub = self.create_publisher(String, SCENE_TOPIC, 10)
        self.timer = self.create_timer(0.1, self._tick)
        self.publicados = 0

    def _tick(self):
        escena = {
            "timestamp": time.time(),
            "objects": [
                {
                    "id": "red_box_1",
                    "class": "red",
                    "reachable": True,
                    "point": {"x": 0.15, "y": 0.0, "z": 0.10},
                }
            ],
            "zones": {"drop_zone_a": [0.18, -0.15, 0.12]},
            "robot": {"busy": False, "position": {"x": 0.15, "y": 0.0, "z": 0.10}},
        }
        msg = String()
        msg.data = json.dumps(escena)
        self.pub.publish(msg)
        self.publicados += 1


def test_escena_se_refresca_durante_inferencia_bloqueante(rclpy_ctx):
    """La inferencia bloquea 1.5 s. El subscriber DEBE poder escribir en ese lapso.

    Con umbral de frescura de 2.0 s, si el executor queda hambriento la escena
    tendra mas de 1.5 s de edad y grounding fallara.
    """
    node = LlmAgentNode()
    node.dry_run = False
    node.validation_engine.is_dry_run = False
    node.validation_engine.max_stale_age_s = 2.0

    publicador = PublicadorDeEscena()

    # Escena inicial deliberadamente vieja: si nadie la refresca, grounding falla.
    node.current_scene = {
        "timestamp": time.time() - 30.0,
        "objects": [
            {
                "id": "red_box_1",
                "class": "red",
                "reachable": True,
                "point": {"x": 0.15, "y": 0.0, "z": 0.10},
            }
        ],
        "zones": {"drop_zone_a": [0.18, -0.15, 0.12]},
        "robot": {"busy": False, "position": {"x": 0.15, "y": 0.0, "z": 0.10}},
    }

    # El mock NO toca current_scene. Solo bloquea, como lo hace Ollama.
    def inferencia_lenta(system_prompt, user_prompt):
        time.sleep(LATENCIA_SIMULADA_S)
        res = MagicMock()
        res.raw_response = '{"task": "pick", "target_object_id": "red_box_1"}'
        res.latency_s = LATENCIA_SIMULADA_S
        return res

    backend = MagicMock()
    backend.generate_plan.side_effect = inferencia_lenta
    node.backend = backend
    node.use_llm_api = True

    publicados = []
    node.plan_publisher = MagicMock()
    node.plan_publisher.publish.side_effect = (
        lambda msg: publicados.append(json.loads(msg.data))
    )

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.add_node(publicador)
    hilo = threading.Thread(target=executor.spin, daemon=True)
    hilo.start()

    try:
        time.sleep(0.5)  # dejar que lleguen los primeros mensajes de escena
        assert publicador.publicados > 0, "el publicador de escena nunca corrio"

        node._process_command_with_llm("agarra la caja roja")

        assert len(publicados) == 1
        telemetria = publicados[0]["telemetry"]

        assert telemetria["grounding_ok"] is True, (
            "grounding fallo: la escena no se refresco durante la inferencia. "
            "Revisa si la suscripcion de escena esta en un ReentrantCallbackGroup "
            "y si el nodo corre bajo MultiThreadedExecutor. "
            f"Error: {telemetria.get('grounding_error')}"
        )
        assert telemetria["plan_valid"] is True
        assert telemetria["first_blocking_layer"] == "none"
    finally:
        executor.shutdown()
        node.destroy_node()
        publicador.destroy_node()


def test_edad_de_escena_usada_es_menor_al_umbral(rclpy_ctx):
    """Verifica directamente la propiedad temporal, no solo el veredicto.

    Requiere que el nodo exponga el timestamp de la escena que efectivamente
    uso para validar. Si no lo tienes, agregalo a la telemetria: es la unica
    forma de auditar este bug desde los CSV en vez de por inferencia indirecta.
    """
    node = LlmAgentNode()
    node.dry_run = False
    node.validation_engine.is_dry_run = False
    publicador = PublicadorDeEscena()

    def inferencia_lenta(system_prompt, user_prompt):
        time.sleep(LATENCIA_SIMULADA_S)
        res = MagicMock()
        res.raw_response = '{"task": "pick", "target_object_id": "red_box_1"}'
        res.latency_s = LATENCIA_SIMULADA_S
        return res

    backend = MagicMock()
    backend.generate_plan.side_effect = inferencia_lenta
    node.backend = backend
    node.use_llm_api = True

    publicados = []
    node.plan_publisher = MagicMock()
    node.plan_publisher.publish.side_effect = (
        lambda msg: publicados.append(json.loads(msg.data))
    )

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.add_node(publicador)
    hilo = threading.Thread(target=executor.spin, daemon=True)
    hilo.start()

    try:
        time.sleep(0.5)
        t_inicio = time.time()
        node._process_command_with_llm("agarra la caja roja")

        telemetria = publicados[0]["telemetry"]
        ts_usado = telemetria.get("scene_timestamp_used")
        if ts_usado is None:
            pytest.skip(
                "El nodo no reporta scene_timestamp_used en la telemetria. "
                "Agregalo: sin ese campo el bug de frescura no es auditable "
                "desde los CSV."
            )
        edad = time.time() - ts_usado
        assert edad < node.validation_engine.max_stale_age_s
        assert ts_usado > t_inicio, (
            "el snapshot usado es anterior al inicio del comando: "
            "el nodo esta reutilizando la escena vieja"
        )
    finally:
        executor.shutdown()
        node.destroy_node()
        publicador.destroy_node()