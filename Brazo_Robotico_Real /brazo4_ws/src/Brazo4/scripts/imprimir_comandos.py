#!/usr/bin/env python3
import sys
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
from sensor_msgs.msg import JointState

class ImprimirComandosNode(Node):
    def __init__(self):
        super().__init__('imprimir_comandos')
        
        # Diccionario para almacenar los últimos comandos recibidos (inicializados en None)
        self.commands = {
            'nema': None,
            'pololu': None,
            'servo1': None,
            'servo2': None,
            'servo3': None
        }
        
        # Suscriptores
        self.sub_nema = self.create_subscription(
            Float32, '/motor_nema/target_deg', self.nema_callback, 10)
        self.sub_pololu = self.create_subscription(
            Float32, '/motor_pololu/target_deg', self.pololu_callback, 10)
        self.sub_servo1 = self.create_subscription(
            Int32, '/servo1/target_deg', self.servo1_callback, 10)
        self.sub_servo2 = self.create_subscription(
            Int32, '/servo2/target_deg', self.servo2_callback, 10)
        self.sub_servo3 = self.create_subscription(
            Int32, '/servo3/target_deg', self.servo3_callback, 10)
            
        # Publicador de JointStates usando un tópico exclusivo para evitar conflictos en RViz
        self.joint_pub = self.create_publisher(JointState, '/joint_states_target', 10)
        
        # Nombres de las articulaciones del URDF brazo4central
        self.joint_names = [
            '1', '2', '3', '4',
            'left_joint', 'right_joint',
            'left_gear', 'right_gear_joint'
        ]
        
        # Publicar JointState a 50Hz para mantener la actualización de TF constante en RViz (sin expirar)
        self.timer = self.create_timer(0.02, self.timer_callback)
        
        self.get_logger().info("Nodo 'imprimir_comandos' iniciado. Escuchando tópicos de control...")
        self.print_status() # Imprimir estado inicial vacío una sola vez

    def map_range(self, val, in_min, in_max, out_min, out_max):
        if in_max == in_min:
            return out_min
        val = max(min(in_min, in_max), min(max(in_min, in_max), val))
        return out_min + (val - in_min) * (out_max - out_min) / (in_max - in_min)

    def print_status(self):
        # Mostrar guiones si el valor es None (aún no se recibe mensaje)
        nema_val = f"{self.commands['nema']:6.1f}°" if self.commands['nema'] is not None else "   --- "
        pololu_val = f"{self.commands['pololu']:6.1f}°" if self.commands['pololu'] is not None else "   --- "
        servo1_val = f"{self.commands['servo1']:4d}°" if self.commands['servo1'] is not None else "  --"
        servo2_val = f"{self.commands['servo2']:4d}°" if self.commands['servo2'] is not None else "  --"
        servo3_val = f"{self.commands['servo3']:4d}°" if self.commands['servo3'] is not None else "  --"

        sys.stdout.write(
            f"\r[Comandos] "
            f"NEMA: {nema_val} | "
            f"Pololu: {pololu_val} | "
            f"Servo 1 (J3): {servo1_val} | "
            f"Servo 2 (J4): {servo2_val} | "
            f"Servo 3 (Gripper): {servo3_val}"
        )
        sys.stdout.flush()

    # Los callbacks actualizan el diccionario e imprimen a consola ÚNICAMENTE cuando llega un nuevo dato
    def nema_callback(self, msg):
        self.commands['nema'] = msg.data
        self.print_status()

    def pololu_callback(self, msg):
        self.commands['pololu'] = msg.data
        self.print_status()

    def servo1_callback(self, msg):
        self.commands['servo1'] = msg.data
        self.print_status()

    def servo2_callback(self, msg):
        self.commands['servo2'] = msg.data
        self.print_status()

    def servo3_callback(self, msg):
        self.commands['servo3'] = msg.data
        self.print_status()

    def timer_callback(self):
        # Mapear los comandos recibidos (usando valores por defecto si no han llegado mensajes)
        nema_deg = self.commands['nema'] if self.commands['nema'] is not None else 0.0
        pololu_deg = self.commands['pololu'] if self.commands['pololu'] is not None else 0.0
        servo1_deg = float(self.commands['servo1']) if self.commands['servo1'] is not None else 90.0
        servo2_deg = float(self.commands['servo2']) if self.commands['servo2'] is not None else 0.0
        servo3_deg = float(self.commands['servo3']) if self.commands['servo3'] is not None else 90.0
        
        # NEMA (J1): 0.0 a 130.0 deg -> 0.0 a -130.0 deg en RViz
        j1_deg = self.map_range(nema_deg, 0.0, 130.0, 0.0, -130.0)
        q_j1 = math.radians(j1_deg)
        
        # Pololu (J2): -320.0 a 0.0 deg -> -140.0 a 0.0 deg en RViz
        j2_deg = self.map_range(pololu_deg, -320.0, 0.0, -140.0, 0.0)
        q_j2 = math.radians(j2_deg)
        
        # Servo 1 (J3): 0.0 a 180.0 deg -> 0.0 a 150.0 deg en RViz
        j3_deg = self.map_range(servo1_deg, 0.0, 180.0, 0.0, 150.0)
        q_j3 = math.radians(j3_deg)
        
        # Servo 2 (J4): 0.0 a 150.0 deg -> 0.0 a 180.0 deg en RViz
        j4_deg = self.map_range(servo2_deg, 0.0, 150.0, 0.0, 180.0)
        q_j4 = math.radians(j4_deg)
        
        # Servo 3 (Gripper): 0.0 a 180.0 deg -> -0.2 a 0.6 rad en RViz
        q_gripper = self.map_range(servo3_deg, 0.0, 180.0, -0.2, 0.6)
        
        # Publicar JointState
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [
            q_j1,
            q_j2,
            q_j3,
            q_j4,
            q_gripper,            # left_joint
            q_gripper,            # right_joint
            0.41 * q_gripper,     # left_gear
            0.41 * q_gripper      # right_gear_joint
        ]
        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImprimirComandosNode()
    
    print("\n====================================================================================================")
    print("                MONITOR Y PUBLICADOR DE OBJETIVOS PARA RVIZ - BRAZO4")
    print("====================================================================================================")
    print(" Escuchando tópicos de control (target_deg) y publicando a /joint_states_target... Ctrl+C para salir.\n")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nMonitoreo finalizado por el usuario.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
