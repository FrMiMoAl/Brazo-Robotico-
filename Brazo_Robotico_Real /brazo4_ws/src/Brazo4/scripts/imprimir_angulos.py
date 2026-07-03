#!/usr/bin/env python3
import sys
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class ImprimirAngulosNode(Node):
    def __init__(self):
        super().__init__('imprimir_angulos')
        
        # Suscribirse al tópico de estados de articulaciones
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )
        
        # Diccionario para almacenar los últimos valores recibidos
        self.joint_positions = {'J1': 0.0, 'J2': 0.0, 'J3': 0.0, 'J4': 0.0}
        self.has_received_data = False

        self.get_logger().info("Nodo 'imprimir_angulos' iniciado. Escuchando /joint_states...")

    def joint_states_callback(self, msg):
        # Mapear nombres de articulaciones ('1', '2', '3', '4' o 'J1', 'J2', 'J3', 'J4')
        for name, position in zip(msg.name, msg.position):
            if name in ['1', 'J1']:
                self.joint_positions['J1'] = position
                self.has_received_data = True
            elif name in ['2', 'J2']:
                self.joint_positions['J2'] = position
                self.has_received_data = True
            elif name in ['3', 'J3']:
                self.joint_positions['J3'] = position
                self.has_received_data = True
            elif name in ['4', 'J4']:
                self.joint_positions['J4'] = position
                self.has_received_data = True

        if self.has_received_data:
            # Obtener valores en grados y radianes
            j1_rad = self.joint_positions['J1']
            j2_rad = self.joint_positions['J2']
            j3_rad = self.joint_positions['J3']
            j4_rad = self.joint_positions['J4']

            j1_deg = math.degrees(j1_rad)
            j2_deg = math.degrees(j2_rad)
            j3_deg = math.degrees(j3_rad)
            j4_deg = math.degrees(j4_rad)

            # Imprimir de forma limpia en la misma línea de la consola usando carriage return (\r)
            # Esto evita saturar la pantalla con saltos de línea continuos.
            sys.stdout.write(
                f"\r[Ángulos] "
                f"J1: {j1_deg:6.1f}° ({j1_rad:6.3f} rad) | "
                f"J2: {j2_deg:6.1f}° ({j2_rad:6.3f} rad) | "
                f"J3: {j3_deg:6.1f}° ({j3_rad:6.3f} rad) | "
                f"J4: {j4_deg:6.1f}° ({j4_rad:6.3f} rad)"
            )
            sys.stdout.flush()

def main(args=None):
    rclpy.init(args=args)
    node = ImprimirAngulosNode()
    
    print("\n==================================================================================")
    # Clear line and print header
    print("                MONITOR DE ÁNGULOS DE ARTICULACIONES - BRAZO4")
    print("==================================================================================")
    print(" Escuchando /joint_states... Presiona Ctrl+C para salir.\n")
    
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
