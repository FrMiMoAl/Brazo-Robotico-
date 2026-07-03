#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import sys
import select
import termios
import tty
import numpy as np

msg = """
============================================================
       TELEOPERACIÓN POR ARTICULACIÓN (MODO EJES)
============================================================
Controla cada articulación individualmente:

   [p]       : Cambiar de articulación (J1 -> J2 -> J3 -> J4)
   [w]       : Incrementar ángulo (+5° / +0.087 rad)
   [s]       : Decrementar ángulo (-5° / -0.087 rad)

   [o]       : ABRIR Gripper (Tijeras)
   [c]       : CERRAR Gripper (Tijeras)
   [barra]   : Restablecer todas las juntas a 0° (Reposada)

   [Ctrl+C]  : Salir del programa
============================================================
"""

# Límites angulares
JOINT_LIMITS = [
    (None, None),                             # J1 (Continua)
    (np.radians(-140), np.radians(0)),        # J2 (-140 a 0)
    (0.0, np.radians(150)),                   # J3 (0 a 150)
    (0.0, np.radians(180))                    # J4 (0 a 180)
]

class TeleopArticular(Node):
    def __init__(self):
        super().__init__('teleop_brazo')
        
        # Publicadores
        self.pub_joints = self.create_publisher(JointState, '/joint_commands', 10)
        self.pub_grip = self.create_publisher(Bool, '/gripper_command', 10)
        
        # Estado de las juntas (J1, J2, J3, J4) en radianes (Home: J1=0.0, J2=0.0, J3=0.0, J4=0.0)
        self.q = [0.0, 0.0, 0.0, 0.0]
        self.active_joint_idx = 0  # Comienza en J1 (índice 0)
        self.gripper_open = False
        
        self.get_logger().info("Nodo de Teleoperación Articular Iniciado.")
        self.print_status()

    def update_joint(self, delta):
        idx = self.active_joint_idx
        q_new = self.q[idx] + delta
        
        # Verificar límites
        if idx == 0:
            # Wrap-around para J1 (Continua)
            q_new = np.arctan2(np.sin(q_new), np.cos(q_new))
            self.q[idx] = q_new
            self.publish_joints()
            self.print_status()
        else:
            # Límites fijos para J2, J3, J4
            low, high = JOINT_LIMITS[idx]
            if low <= q_new <= high:
                self.q[idx] = q_new
                self.publish_joints()
                self.print_status()
            else:
                self.print_status(warning=f"¡LÍMITE! Articulación J{idx+1} alcanzó su rango máximo ({np.degrees(low):.1f}° a {np.degrees(high):.1f}°).")

    def cycle_joint(self):
        self.active_joint_idx = (self.active_joint_idx + 1) % 4
        self.print_status()

    def publish_joints(self):
        msg_joints = JointState()
        msg_joints.header.stamp = self.get_clock().now().to_msg()
        msg_joints.name = ['J1', 'J2', 'J3', 'J4']
        msg_joints.position = [float(val) for val in self.q]
        self.pub_joints.publish(msg_joints)

    def set_gripper(self, open_state):
        self.gripper_open = open_state
        msg_bool = Bool()
        msg_bool.data = bool(open_state)
        self.pub_grip.publish(msg_bool)
        self.print_status()

    def reset_pose(self):
        self.q = [0.0, 0.0, 0.0, 0.0]
        self.publish_joints()
        self.print_status(warning="Todas las articulaciones regresaron al Home.")

    def print_status(self, warning=""):
        # Limpiar línea y escribir el estado formateado
        sys.stdout.write("\033[K")
        
        joint_strs = []
        for i in range(4):
            deg = np.degrees(self.q[i])
            if i == self.active_joint_idx:
                joint_strs.append(f"\033[1;36m*J{i+1}: {deg:6.1f}°*\033[0m")
            else:
                joint_strs.append(f"J{i+1}: {deg:6.1f}°")
                
        status_line = " | ".join(joint_strs)
        sys.stdout.write(f"\r[Estado] {status_line} | Gripper: {'ABIERTO' if self.gripper_open else 'CERRADO'}")
        if warning:
            sys.stdout.write(f"\n\033[31m[ADVERTENCIA] {warning}\033[0m\n")
        sys.stdout.flush()

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    settings = termios.tcgetattr(sys.stdin)
    
    rclpy.init()
    node = TeleopArticular()
    
    print(msg)
    
    # Delta de incremento (5 grados en radianes)
    delta_angle = np.radians(5.0)
    
    try:
        while True:
            key = get_key(settings)
            if not key:
                continue
                
            if key == 'p':     # Cambiar de articulación
                node.cycle_joint()
            elif key == 'w':   # Incrementar articulación activa
                node.update_joint(delta_angle)
            elif key == 's':   # Decrementar articulación activa
                node.update_joint(-delta_angle)
            elif key == 'o':   # Abrir Gripper
                node.set_gripper(True)
            elif key == 'c':   # Cerrar Gripper
                node.set_gripper(False)
            elif key == ' ':   # Reset
                node.reset_pose()
            elif key == '\x03': # Ctrl+C
                break
                
    except Exception as e:
        print(e)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
