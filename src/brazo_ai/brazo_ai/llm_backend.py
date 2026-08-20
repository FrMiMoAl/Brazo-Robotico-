from dataclasses import dataclass
from time import perf_counter

try:
    from ollama import Client
except ImportError:
    Client = None

from .plan_schema import RobotPlan


@dataclass(frozen=True)
class InferenceResult:
    plan: Optional[RobotPlan]
    raw_response: str
    model: str
    latency_s: float
    input_tokens: int | None
    output_tokens: int | None


class OllamaBackend:
    def __init__(
        self,
        model: str,
        host: str = "http://localhost:11434",
        timeout_s: float = 30.0,
    ) -> None:
        if Client is None:
            raise ImportError(
                "El paquete 'ollama' no esta instalado en Python. Instalalo usando 'pip install ollama'."
            )
        self.model = model
        self.client = Client(host=host, timeout=timeout_s)


    def generate_plan(
        self,
        system_prompt: str,
        user_prompt: str,
    ) -> InferenceResult:
        started_at = perf_counter()

        response = self.client.chat(
            model=self.model,
            messages=[
                {
                    "role": "system",
                    "content": system_prompt,
                },
                {
                    "role": "user",
                    "content": user_prompt,
                },
            ],
            format=RobotPlan.model_json_schema(),
            stream=False,
            options={
                "temperature": 0,
                "seed": 42,
                "num_predict": 256,
            },
        )

        latency_s = perf_counter() - started_at
        raw_text = getattr(response.message, "content", "") or str(response.message)

        plan = None
        try:
            plan = RobotPlan.model_validate_json(raw_text)
        except Exception:
            pass

        return InferenceResult(
            plan=plan,
            raw_response=raw_text,
            model=self.model,
            latency_s=latency_s,
            input_tokens=getattr(
                response,
                "prompt_eval_count",
                None,
            ),
            output_tokens=getattr(
                response,
                "eval_count",
                None,
            ),
        )
