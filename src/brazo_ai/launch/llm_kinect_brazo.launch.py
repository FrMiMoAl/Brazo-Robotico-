#!/usr/bin/env python3
"""Lanza la capa de IA segura: percepcion->base_link, scene_state, planificador,
executor y safety guard.

NO lanza kinect2_bridge ni object_3d_detector (tienen su propio launch / se
corren aparte) ni ningun controlador de hardware del brazo: este launch
file se inserta POR ENCIMA de lo que ya existe, sin tocarlo.

Arranca seguro por defecto:
    dry_run:=true autonomous_enable:=false hardware_armed:=false

Uso tipico (simulado, sin tocar el brazo):
    ros2 launch brazo_ai llm_kinect_brazo.launch.py dry_run:=true

Uso con brazo real (solo despues de validar todo en dry_run y calibrar TF):
    ros2 launch brazo_ai llm_kinect_brazo.launch.py \\
        dry_run:=false autonomous_enable:=true hardware_armed:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declared_args = [
        # --- Seguridad: SIEMPRE empezar en modo seguro ---
        DeclareLaunchArgument("dry_run", default_value="true",
                               description="Si es true, safety_guard_node y task_executor_node solo simulan/loguean, nunca mueven el brazo real."),
        DeclareLaunchArgument("autonomous_enable", default_value="false",
                               description="Debe ser true ademas de dry_run=false para permitir movimiento real."),
        DeclareLaunchArgument("hardware_armed", default_value="false",
                               description="Debe ser true ademas de dry_run=false para permitir movimiento real."),

        # --- Percepcion / clase objetivo ---
        # NOTA: este launch no lanza object_3d_detector. Si se quiere que
        # el detector use la misma clase, pasar el mismo valor con
        # 'ros2 run object_3d_detector yolo_depth_to_point --ros-args -p target_class:=<valor>'.
        DeclareLaunchArgument("target_class", default_value="red",
                               description="Clase de objeto objetivo (informativo; el detector se lanza aparte)."),

        # --- Frames ---
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("camera_frame", default_value="kinect2_depth_optical_frame"),

        # --- Workspace del brazo (PLACEHOLDERS, medir y ajustar, ver FASE 5 / README) ---
        DeclareLaunchArgument("workspace_x_min", default_value="0.05"),
        DeclareLaunchArgument("workspace_x_max", default_value="0.35"),
        DeclareLaunchArgument("workspace_y_min", default_value="-0.25"),
        DeclareLaunchArgument("workspace_y_max", default_value="0.25"),
        DeclareLaunchArgument("workspace_z_min", default_value="0.03"),
        DeclareLaunchArgument("workspace_z_max", default_value="0.35"),
        DeclareLaunchArgument("max_step_m", default_value="0.12",
                               description="Salto cartesiano maximo permitido por safety_guard_node entre comandos consecutivos; task_executor_node parte movimientos largos en waypoints de este tamano."),

        # --- Zonas conocidas (PLACEHOLDERS, ajustar al brazo real) ---
        DeclareLaunchArgument("zone_home", default_value="[0.16, 0.0, 0.20]"),
        DeclareLaunchArgument("zone_drop_zone_a", default_value="[0.18, -0.15, 0.12]"),
        DeclareLaunchArgument("zone_drop_zone_b", default_value="[0.18, 0.15, 0.12]"),

        # --- Planificador: llm_agent_node (modo determinista por defecto,
        # use_llm_api:=true para conectar a la API de Anthropic) o
        # manual_plan_node (solo pruebas por parametro 'task' / topico) ---
        DeclareLaunchArgument("use_llm_agent", default_value="true",
                               description="true: lanza llm_agent_node. false: lanza manual_plan_node."),
        DeclareLaunchArgument("use_llm_api", default_value="false",
                               description="Solo aplica si use_llm_agent=true. Requiere ANTHROPIC_API_KEY en el entorno."),
        DeclareLaunchArgument("llm_model", default_value="claude-sonnet-5"),
        DeclareLaunchArgument("manual_task", default_value="",
                               description="Solo aplica si use_llm_agent=false. Ej: pick_red, go_home, open_gripper."),
        DeclareLaunchArgument("default_place_zone", default_value="drop_zone_a"),

        # --- Secuencia cartesiana del executor ---
        DeclareLaunchArgument("pregrasp_dz", default_value="0.08"),
        DeclareLaunchArgument("grasp_dz", default_value="0.015"),
        DeclareLaunchArgument("lift_dz", default_value="0.12"),
        DeclareLaunchArgument("move_wait_s", default_value="2.0"),
        DeclareLaunchArgument("gripper_wait_s", default_value="1.0"),

        DeclareLaunchArgument("object_stale_timeout", default_value="0.75"),
        DeclareLaunchArgument("publish_rate_hz", default_value="2.0"),
    ]

    base_frame = LaunchConfiguration("base_frame")
    camera_frame = LaunchConfiguration("camera_frame")
    dry_run = LaunchConfiguration("dry_run")
    autonomous_enable = LaunchConfiguration("autonomous_enable")
    hardware_armed = LaunchConfiguration("hardware_armed")
    workspace_params = {
        "workspace_x_min": LaunchConfiguration("workspace_x_min"),
        "workspace_x_max": LaunchConfiguration("workspace_x_max"),
        "workspace_y_min": LaunchConfiguration("workspace_y_min"),
        "workspace_y_max": LaunchConfiguration("workspace_y_max"),
        "workspace_z_min": LaunchConfiguration("workspace_z_min"),
        "workspace_z_max": LaunchConfiguration("workspace_z_max"),
    }

    camera_to_base_node = Node(
        package="brazo_ai",
        executable="camera_to_base_node",
        name="camera_to_base_node",
        output="screen",
        parameters=[{
            "base_frame": base_frame,
            "camera_frame": camera_frame,
            "object_stale_timeout": LaunchConfiguration("object_stale_timeout"),
            **workspace_params,
        }],
    )

    scene_state_node = Node(
        package="brazo_ai",
        executable="scene_state_node",
        name="scene_state_node",
        output="screen",
        parameters=[{
            "base_frame": base_frame,
            "camera_frame": camera_frame,
            "publish_rate_hz": LaunchConfiguration("publish_rate_hz"),
            "hardware_armed": hardware_armed,
            "zone_home": LaunchConfiguration("zone_home"),
            "zone_drop_zone_a": LaunchConfiguration("zone_drop_zone_a"),
            "zone_drop_zone_b": LaunchConfiguration("zone_drop_zone_b"),
        }],
    )

    llm_agent_node = Node(
        package="brazo_ai",
        executable="llm_agent_node",
        name="llm_agent_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_llm_agent")),
        parameters=[{
            "use_llm_api": LaunchConfiguration("use_llm_api"),
            "llm_model": LaunchConfiguration("llm_model"),
            "default_place_zone": LaunchConfiguration("default_place_zone"),
        }],
    )

    manual_plan_node = Node(
        package="brazo_ai",
        executable="manual_plan_node",
        name="manual_plan_node",
        output="screen",
        condition=UnlessCondition(LaunchConfiguration("use_llm_agent")),
        parameters=[{
            "task": LaunchConfiguration("manual_task"),
            "default_place_zone": LaunchConfiguration("default_place_zone"),
        }],
    )

    task_executor_node = Node(
        package="brazo_ai",
        executable="task_executor_node",
        name="task_executor_node",
        output="screen",
        parameters=[{
            "base_frame": base_frame,
            "dry_run": dry_run,
            "max_step_m": LaunchConfiguration("max_step_m"),
            "pregrasp_dz": LaunchConfiguration("pregrasp_dz"),
            "grasp_dz": LaunchConfiguration("grasp_dz"),
            "lift_dz": LaunchConfiguration("lift_dz"),
            "move_wait_s": LaunchConfiguration("move_wait_s"),
            "gripper_wait_s": LaunchConfiguration("gripper_wait_s"),
            "default_place_zone": LaunchConfiguration("default_place_zone"),
        }],
    )

    safety_guard_node = Node(
        package="brazo_ai",
        executable="safety_guard_node",
        name="safety_guard_node",
        output="screen",
        parameters=[{
            "base_frame": base_frame,
            "dry_run": dry_run,
            "autonomous_enable": autonomous_enable,
            "hardware_armed": hardware_armed,
            "max_step_m": LaunchConfiguration("max_step_m"),
            **workspace_params,
        }],
    )

    return LaunchDescription(declared_args + [
        camera_to_base_node,
        scene_state_node,
        llm_agent_node,
        manual_plan_node,
        task_executor_node,
        safety_guard_node,
    ])
