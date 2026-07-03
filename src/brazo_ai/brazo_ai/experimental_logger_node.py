#!/usr/bin/env python3
"""
experimental_logger_node.py

Nodo diseñado para recopilar métricas experimentales de la arquitectura de robótica
(LLM, Perception, Task Executor, Safety Guard) y guardarlas en un archivo CSV.
Ideal para validación científica en papers (ej. medición de latencia, exactitud
de calibración mediante Ground Truth manual, y robustez de la capa de seguridad).
"""

import time
import json
import csv
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from brazo_interfaces.msg import ArmCommandStatus

class ExperimentalLoggerNode(Node):
    def __init__(self):
        super().__init__('experimental_logger_node')
        
        # Parámetros
        self.declare_parameter('log_file', os.path.expanduser('~/ros2_ws2/experimental_metrics.csv'))
        self.log_file = self.get_parameter('log_file').value
        
        # Inicializar CSV si no existe
        self._init_csv()
        
        # Estado interno
        self._last_user_command_time = None
        self._last_plan_start_time = None
        self._last_cam_point = None
        
        # Suscriptores
        self.create_subscription(String, '/user_command', self.user_command_cb, 10)
        self.create_subscription(String, '/llm_plan', self.llm_plan_cb, 10)
        self.create_subscription(PointStamped, '/perception/selected_object_camera', self.camera_cb, 10)
        self.create_subscription(PointStamped, '/perception/selected_object_base', self.base_cb, 10)
        self.create_subscription(ArmCommandStatus, '/arm/command_status', self.status_cb, 10)

        self.get_logger().info(f"Logger experimental iniciado. Guardando datos en: {self.log_file}")
        self.get_logger().info(f"ADVERTENCIA: Las columnas GT_X, GT_Y, GT_Z quedan vacías para ingreso manual.")

    def _init_csv(self):
        file_exists = os.path.isfile(self.log_file)
        with open(self.log_file, mode='a', newline='') as f:
            writer = csv.writer(f)
            if not file_exists:
                writer.writerow(['Timestamp', 'Layer', 'Event', 'Data', 'Status', 'GT_X', 'GT_Y', 'GT_Z'])

    def _log_event(self, layer, event, data, status):
        timestamp = time.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        data_str = json.dumps(data) if isinstance(data, dict) else str(data)
        with open(self.log_file, mode='a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([timestamp, layer, event, data_str, status, '', '', ''])

    def user_command_cb(self, msg: String):
        self._last_user_command_time = time.time()
        self._log_event('LLM', 'UserCommandReceived', {'command': msg.data}, 'VALID')

    def llm_plan_cb(self, msg: String):
        latency = -1.0
        if self._last_user_command_time is not None:
            latency = time.time() - self._last_user_command_time
            self._last_user_command_time = None
        
        try:
            plan = json.loads(msg.data)
            status = 'VALID'
            task = plan.get('task', 'unknown')
        except json.JSONDecodeError:
            plan = "invalid_json"
            status = 'INVALID'
            task = 'unknown'

        self._log_event('LLM', 'PlanGenerated', {'latency_s': round(latency, 3), 'task': task, 'plan': plan}, status)

    def camera_cb(self, msg: PointStamped):
        self._last_cam_point = (msg.point.x, msg.point.y, msg.point.z)

    def base_cb(self, msg: PointStamped):
        base_pt = (msg.point.x, msg.point.y, msg.point.z)
        if self._last_cam_point:
            cam_pt = self._last_cam_point
            data = {
                'cam_x': round(cam_pt[0], 3), 'cam_y': round(cam_pt[1], 3), 'cam_z': round(cam_pt[2], 3),
                'base_x': round(base_pt[0], 3), 'base_y': round(base_pt[1], 3), 'base_z': round(base_pt[2], 3)
            }
            self._log_event('Perception', 'CoordinateTransformed', data, 'SUCCESS')
            self._last_cam_point = None

    def status_cb(self, msg: ArmCommandStatus):
        # 1. Deterministic sequence validation (Task Executor)
        if msg.state == 'LOAD_PLAN':
            self._last_plan_start_time = time.time()
            self._log_event('Executor', 'SequenceStarted', {'message': msg.message}, 'IN_PROGRESS')
        elif msg.state == 'DONE' and self._last_plan_start_time is not None:
            latency = time.time() - self._last_plan_start_time
            self._last_plan_start_time = None
            self._log_event('Executor', 'SequenceCompleted', {'latency_s': round(latency, 3), 'message': msg.message}, 'SUCCESS')

        # 2. Safety guard checks
        if msg.message.startswith('safety_guard:'):
            reason = msg.message.replace('safety_guard:', '').strip()
            data = {'reason': reason}
            if msg.current_target and msg.current_target.header.frame_id != '':
                data['target'] = {'x': round(msg.current_target.point.x, 3), 
                                  'y': round(msg.current_target.point.y, 3), 
                                  'z': round(msg.current_target.point.z, 3)}
            
            # Note: The DRY_RUN state acts functionally as ACCEPTED but without HW movement
            if msg.state == 'ACCEPTED' or msg.state == 'DRY_RUN':
                self._log_event('SafetyGuard', 'CommandEvaluated', data, 'ACCEPTED')
            elif msg.state == 'REJECTED':
                self._log_event('SafetyGuard', 'CommandEvaluated', data, 'REJECTED')

def main(args=None):
    rclpy.init(args=args)
    node = ExperimentalLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
