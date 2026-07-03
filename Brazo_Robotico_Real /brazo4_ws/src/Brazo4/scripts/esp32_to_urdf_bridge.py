#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
from std_msgs.msg import Float32, Int32
from sensor_msgs.msg import JointState

class ESP32ToURDFBridge(Node):
    def __init__(self):
        super().__init__('esp32_to_urdf_bridge')
        
        # Ángulos actuales en RViz (Radianes) - Estado interno de suavizado
        self.current_angles = {'1': 0.0, '2': 0.0, '3': 0.0, '4': 0.0, 'gripper': 0.0}
        
        # Ángulos objetivo (Targets) basados en tu hardware real
        self.target_angles = {'1': 0.0, '2': 0.0, '3': 0.0, '4': 0.0, 'gripper': 0.0}
        
        # VELOCIDAD DEL MOVIMIENTO SUAVE (Radianes por segundo)
        self.joint_speed = 1.0 
        
        # Publicador unificado hacia /joint_states
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # SUSCRIPTORES CON ASIGNACIÓN REAL DE TU HARDWARE
        self.create_subscription(Float32, '/motor_nema/target_deg', self.nema_cb, 10)     # j1 -> NEMA
        self.create_subscription(Float32, '/motor_pololu/target_deg', self.pololu_cb, 10) # j2 -> Pololu
        self.create_subscription(Int32, '/servo2/target_deg', self.servo2_cb, 10)         # j3 -> Servo 2
        self.create_subscription(Int32, '/servo1/target_deg', self.servo1_cb, 10)         # j4 -> Servo 1
        self.create_subscription(Int32, '/servo3/target_deg', self.servo3_cb, 10)         # Gripper -> Servo 3
        
        # Nombres de las articulaciones del URDF brazo4central
        self.joint_names = [
            '1', '2', '3', '4',
            'left_joint', 'right_joint',
            'left_gear', 'right_gear_joint'
        ]
        
        # Temporizador cíclico a 50Hz (dt = 0.02s)
        self.dt = 0.02
        self.create_timer(self.dt, self.timer_callback)
        self.get_logger().info("¡Puente NEMA corregido y activo!")

    def map_range(self, val, in_min, in_max, out_min, out_max):
        if in_max == in_min:
            return out_min
        val = max(min(in_min, in_max), min(max(in_min, in_max), val))
        return out_min + (val - in_min) * (out_max - out_min) / (in_max - in_min)

    # --- CALLBACKS DE TUS MOTORES ---

    def nema_cb(self, msg):
        try:
            # 1. Mapeamos primero el valor crudo en el rango positivo (0.0 a 130.0)
            j1_deg = self.map_range(float(msg.data), 0.0, 130.0, 0.0, 130.0)
            
            # 2. Aplicamos la inversión y el signo negativo al final para RViz
            self.target_angles['1'] = math.radians(j1_deg * -1.0)
        except Exception as e:
            self.get_logger().error(f"Error en NEMA: {e}")

    def pololu_cb(self, msg):
        try:
            j2_deg = self.map_range(float(msg.data), -320.0, 0.0, -140.0, 0.0)
            self.target_angles['2'] = math.radians(j2_deg)
        except Exception as e:
            self.get_logger().error(f"Error en Pololu: {e}")

    def servo2_cb(self, msg):
        try:
            j3_deg = self.map_range(float(msg.data), 0.0, 150.0, 0.0, 180.0)
            self.target_angles['3'] = math.radians(j3_deg)
        except Exception as e:
            self.get_logger().error(f"Error en Servo 2 (j3): {e}")

    def servo1_cb(self, msg):
        try:
            j4_deg = self.map_range(float(msg.data), 0.0, 180.0, 0.0, 180.0)
            self.target_angles['4'] = math.radians(j4_deg)
        except Exception as e:
            self.get_logger().error(f"Error en Servo 1 (j4): {e}")

    def servo3_cb(self, msg):
        try:
            self.target_angles['gripper'] = self.map_range(float(msg.data), 0.0, 180.0, -0.2, 0.6)
        except Exception as e:
            self.get_logger().error(f"Error en Servo 3 (Gripper): {e}")

    # --- CONTROL DE MOVIMIENTO SUAVE ---

    def timer_callback(self):
        try:
            max_step = self.joint_speed * self.dt
            
            for joint in ['1', '2', '3', '4', 'gripper']:
                error = self.target_angles[joint] - self.current_angles[joint]
                if abs(error) > max_step:
                    self.current_angles[joint] += math.copysign(max_step, error)
                else:
                    self.current_angles[joint] = self.target_angles[joint]

            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self.joint_names
            msg.position = [
                self.current_angles['1'],
                self.current_angles['2'],
                self.current_angles['3'],
                self.current_angles['4'],
                self.current_angles['gripper'],          
                self.current_angles['gripper'],          
                0.41 * self.current_angles['gripper'],   
                0.41 * self.current_angles['gripper']    
            ]
            self.joint_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"Error en ciclo de control suave: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ESP32ToURDFBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()