#!/usr/bin/env python3
import os
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import numpy as np
import select
import termios
import tty
from ament_index_python.packages import get_package_share_directory

msg = """
============================================================
       TELEOPERACIÓN CARTESIANA (ESPACIO DE COORDENADAS)
============================================================
Controla la posición del extremo (Efector Final) en 3D:

   Teclas de Movimiento (pasos de 1 cm):
      [w] / [s]   : X + / - (Adelante / Atrás)
      [a] / [d]   : Y + / - (Izquierda / Derecha)
      [r] / [f]   : Z + / - (Arriba / Abajo)

   Especiales:
      [barra]     : Reset a posición inicial
      [Ctrl+C]    : Salir del programa
============================================================
"""

# Helper functions for kinematics of arbitrary joint chain
def rot_axis(axis, theta):
    axis = np.array(axis, dtype=float)
    norm = np.linalg.norm(axis)
    if norm > 1e-6:
        axis = axis / norm
    ux, uy, uz = axis
    cos = np.cos(theta)
    sin = np.sin(theta)
    one_cos = 1.0 - cos
    return np.array([
        [cos + ux**2 * one_cos, ux*uy*one_cos - uz*sin, ux*uz*one_cos + uy*sin],
        [uy*ux*one_cos + uz*sin, cos + uy**2 * one_cos, uy*uz*one_cos - ux*sin],
        [uz*ux*one_cos - uy*sin, uz*uy*one_cos + ux*sin, cos + uz**2 * one_cos]
    ])

def rpy_matrix(r, p, y):
    cR, sR = np.cos(r), np.sin(r)
    cP, sP = np.cos(p), np.sin(p)
    cY, sY = np.cos(y), np.sin(y)
    
    Rx = np.array([
        [1, 0, 0],
        [0, cR, -sR],
        [0, sR, cR]
    ])
    Ry = np.array([
        [cP, 0, sP],
        [0, 1, 0],
        [-sP, 0, cP]
    ])
    Rz = np.array([
        [cY, -sY, 0],
        [sY, cY, 0],
        [0, 0, 1]
    ])
    return Rz @ Ry @ Rx

def joint_transform(xyz, rpy, axis, q):
    T = np.eye(4)
    T[:3, 3] = xyz
    T[:3, :3] = rpy_matrix(*rpy) @ rot_axis(axis, q)
    return T

class RobotKinematicsURDF:
    def __init__(self, urdf_path=None):
        # Definición de las articulaciones según el nuevo URDF sin gripper
        self.joints_data = [
            {
                'xyz': [0.045435, -0.019492, 0.042792],
                'rpy': [0.0, 0.0, -1.01533],
                'axis': [0.0, 0.0, 1.0]
            },
            {
                'xyz': [0.0, 0.0, 0.093058],
                'rpy': [1.2140837, -1.0185975, 2.4015789],
                'axis': [0.5, -0.86603, 0.0]
            },
            {
                'xyz': [0.11856, 0.068452, 0.37687],
                'rpy': [0.0, 1.260502, 0.5236],
                'axis': [0.0, -1.0, 0.0]
            },
            {
                'xyz': [0.2336495, 0.0, -0.0478555],
                'rpy': [0.0, -1.0820763, 0.0],
                'axis': [0.0, 1.0, 0.0]
            }
        ]
        # Límites angulares para verificar validez de la solución
        self.joint_limits = [
            (-np.pi, np.pi),                          # J1
            (np.radians(-140), np.radians(0)),        # J2 (-140 a 0)
            (0.0, np.radians(150)),                   # J3 (0 a 150)
            (0.0, np.radians(180))                    # J4 (0 a 180)
        ]

    def forward_kinematics_ee(self, q):
        T = np.eye(4)
        for i in range(4):
            jd = self.joints_data[i]
            T_joint = joint_transform(jd['xyz'], jd['rpy'], jd['axis'], q[i])
            T = T @ T_joint
        
        T_tip = np.eye(4)
        T_tip[0, 3] = 0.08
        T = T @ T_tip
        return T[:3, 3]

    def cinematica_inversa(self, x, y, z, q_init, max_iters=150, tol=1e-4):
        q = np.array(q_init, dtype=float).copy()
        target_pos = np.array([x, y, z])
        damping = 0.005
        
        for i in range(max_iters):
            q[0] = np.arctan2(np.sin(q[0]), np.cos(q[0]))
            
            pos = self.forward_kinematics_ee(q)
            error = target_pos - pos
            
            if np.linalg.norm(error) < tol:
                # Comprobar límites
                for j in range(4):
                    low, high = self.joint_limits[j]
                    if not (low - 1e-3 <= q[j] <= high + 1e-3):
                        raise ValueError("Fuera de límites articulares")
                q[0] = np.arctan2(np.sin(q[0]), np.cos(q[0]))
                return q
            
            # Jacobian
            J = np.zeros((3, 4))
            epsilon = 1e-6
            for j in range(4):
                q_perturbed = q.copy()
                q_perturbed[j] += epsilon
                pos_perturbed = self.forward_kinematics_ee(q_perturbed)
                J[:, j] = (pos_perturbed - pos) / epsilon
                
            JJt = J @ J.T
            J_pseudo = J.T @ np.linalg.inv(JJt + damping * np.eye(3))
            dq = J_pseudo @ error
            
            q += dq
            
            # Limitar durante la búsqueda para evitar divergir (solo joints 2, 3, 4 ya que J1 es continua)
            for j in range(1, 4):
                low, high = self.joint_limits[j]
                q[j] = np.clip(q[j], low, high)
                
        raise ValueError("Fuera de alcance o no converge")

class TeleopCartesiano(Node):
    def __init__(self, robot_kinematics):
        super().__init__('teleop_cartesiano')
        self.kin = robot_kinematics
        self.pub_joints = self.create_publisher(JointState, '/joint_commands', 10)
        self.pub_grip = self.create_publisher(Bool, '/gripper_command', 10)
        
        # Posición inicial de las articulaciones (home: J1=0, J2=0, J3=0.0, J4=0.0)
        self.q = [0.0, 0.0, 0.0, 0.0]
        p_init = self.kin.forward_kinematics_ee(self.q)
        self.x = p_init[0]
        self.y = p_init[1]
        self.z = p_init[2]
        self.gripper_open = False
        
        self.get_logger().info("Nodo de Teleoperación Cartesiana Iniciado.")

    def update_position(self, dx, dy, dz):
        new_x = self.x + dx
        new_y = self.y + dy
        new_z = self.z + dz
        
        try:
            q = self.kin.cinematica_inversa(new_x, new_y, new_z, self.q)
            
            self.q = list(q)
            self.x = new_x
            self.y = new_y
            self.z = new_z
            
            # Publicar
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = ['J1', 'J2', 'J3', 'J4']
            msg.position = [float(val) for val in q]
            self.pub_joints.publish(msg)
            
            self.print_status()
            
        except ValueError as e:
            self.print_status(warning=f"¡FUERA DE ALCANCE! {str(e)}")

    def set_gripper(self, open_state):
        self.gripper_open = open_state
        msg = Bool()
        msg.data = bool(open_state)
        self.pub_grip.publish(msg)
        self.print_status()

    def reset_pose(self):
        self.q = [np.radians(-72.5), np.radians(40.0), np.radians(40.0), np.radians(-90.0)]
        p_init = self.kin.forward_kinematics_ee(self.q)
        self.x = p_init[0]
        self.y = p_init[1]
        self.z = p_init[2]
        
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['J1', 'J2', 'J3', 'J4']
        msg.position = [float(val) for val in self.q]
        self.pub_joints.publish(msg)
        
        self.print_status(warning="Posición inicial restablecida (Home).")

    def print_status(self, warning=""):
        sys.stdout.write("\033[K")
        joint_deg = [np.degrees(val) for val in self.q]
        sys.stdout.write(f"\r[Estado] EE Pos: X={self.x:.3f}m | Y={self.y:.3f}m | Z={self.z:.3f}m | Joints: J1={joint_deg[0]:.1f}°, J2={joint_deg[1]:.1f}°, J3={joint_deg[2]:.1f}°, J4={joint_deg[3]:.1f}°")
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
    kin = RobotKinematicsURDF()
    
    rclpy.init()
    node = TeleopCartesiano(kin)
    
    # Enviar comando de inicialización
    node.update_position(0.0, 0.0, 0.0)
    
    print(msg)
    
    step_xyz = 0.01  # Paso de 1 cm
    
    try:
        while True:
            key = get_key(settings)
            if not key:
                continue
                
            if key == 'w':      # X +
                node.update_position(step_xyz, 0.0, 0.0)
            elif key == 's':    # X -
                node.update_position(-step_xyz, 0.0, 0.0)
            elif key == 'a':    # Y +
                node.update_position(0.0, step_xyz, 0.0)
            elif key == 'd':    # Y -
                node.update_position(0.0, -step_xyz, 0.0)
            elif key == 'r':    # Z +
                node.update_position(0.0, 0.0, step_xyz)
            elif key == 'f':    # Z -
                node.update_position(0.0, 0.0, -step_xyz)
            elif key == ' ':    # Reset
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
