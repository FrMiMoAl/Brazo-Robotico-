#!/bin/bash
# Script automatizado para ejecutar benchmarks en todos los modelos Ollama disponibles

# Cargar entorno de ROS 2 si existe en el workspace actual
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
fi

MODELS=(
  "qwen3:4b-instruct"
  "qwen3:8b"
  "ministral-3:3b"
  "llama3.2:3b"
  "qwen2.5:7b"
  "qwen2.5:3b"
  "qwen2.5-coder:3b"
  "qwen2.5-coder:7b"
  "qwen2.5-coder:14b"
)

echo "=================================================="
echo "  INICIANDO EVALUACION AUTOMATIZADA MULTI-MODELO"
echo "  Total de modelos a evaluar: ${#MODELS[@]}"
echo "=================================================="

for MODEL in "${MODELS[@]}"; do
    CLEAN_NAME=$(echo "$MODEL" | tr '.:-' '_')
    CSV_OUTPUT="results_${CLEAN_NAME}.csv"

    echo ""
    echo "--------------------------------------------------"
    echo ">> [$(date +'%H:%M:%S')] Evaluando Modelo: $MODEL"
    echo ">> Guardando en: $CSV_OUTPUT"
    echo "--------------------------------------------------"

    # 1. Lanzar nodos de IA en segundo plano
    ros2 launch brazo_ai llm_kinect_brazo.launch.py model:="$MODEL" dry_run:=true use_llm_agent:=true use_llm_api:=true > /dev/null 2>&1 &
    LAUNCH_PID=$!

    # 2. Esperar estabilizacion de nodos (5 segundos)
    sleep 5

    # 3. Ejecutar Benchmark Runner
    ros2 run brazo_ai benchmark_runner --ros-args -p output_csv:="$CSV_OUTPUT"

    # 4. Detener Launch y limpiar procesos ROS
    echo "Finalizando proceso launch PID ($LAUNCH_PID)..."
    kill -SIGINT "$LAUNCH_PID" 2>/dev/null
    sleep 2
    kill -9 "$LAUNCH_PID" 2>/dev/null

    # Limpiar nodos por seguridad
    pkill -f "llm_agent_node" 2>/dev/null
    pkill -f "safety_guard_node" 2>/dev/null
    pkill -f "scene_state_node" 2>/dev/null
    pkill -f "task_executor_node" 2>/dev/null
    pkill -f "camera_to_base_node" 2>/dev/null

    sleep 3
done

echo ""
echo "=================================================="
echo "  ¡EVALUACION COMPLETADA PARA TODOS LOS MODELOS!"
echo "=================================================="
