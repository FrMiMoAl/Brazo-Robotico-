#!/usr/bin/env python3
import os
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import numpy as np
from ament_index_python.packages import get_package_share_directory

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
                'rpy': [0.0, 2.0595163, 0.0],
                'axis': [0.0, -1.0, 0.0]
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

class ControlEspacialNode(Node):
    def __init__(self, robot_kinematics):
        super().__init__('mover_coordenadas')
        self.kin = robot_kinematics
        self.pub_joints = self.create_publisher(JointState, '/joint_commands', 10)
        self.pub_grip = self.create_publisher(Bool, '/gripper_command', 10)
        
        self.q_current = [0.0, 0.0, 0.0, np.radians(180.0)]
        self.sub_joints = self.create_subscription(
            JointState,
            '/joint_states',
            self.joints_callback,
            10
        )
        
        self.get_logger().info("Nodo 'mover_coordenadas' iniciado exitosamente.")

    def joints_callback(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if name == 'J1':
                self.q_current[0] = pos
            elif name == 'J2':
                self.q_current[1] = pos
            elif name == 'J3':
                self.q_current[2] = pos
            elif name == 'J4':
                self.q_current[3] = pos

    def mover_a_coordenadas(self, x, y, z):
        try:
            q = self.kin.cinematica_inversa(x, y, z, self.q_current)
            
            # Publicar JointState
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = ['J1', 'J2', 'J3', 'J4']
            msg.position = [float(val) for val in q]
            self.pub_joints.publish(msg)
            
            print(f"\n[Éxito] Comando enviado a /joint_commands:")
            print(f"  -> Coordenadas: X={x:.3f}m, Y={y:.3f}m, Z={z:.3f}m")
            print(f"  -> Ángulos: J1={np.degrees(q[0]):.1f}°, J2={np.degrees(q[1]):.1f}°, J3={np.degrees(q[2]):.1f}°, J4={np.degrees(q[3]):.1f}°")
            return True
            
        except ValueError as e:
            self.get_logger().error(f"Error de cinemática inversa: {e}")
            print("\n[Error] El punto está fuera del alcance del robot (límite matemático).")
            return False

    def enviar_gripper(self, abrir):
        msg = Bool()
        msg.data = bool(abrir)
        self.pub_grip.publish(msg)
        print(f"[Gripper] Comando enviado: {'ABRIR' if abrir else 'CERRAR'}")

def main(args=None):
    rclpy.init(args=args)
    kin = RobotKinematicsURDF()
    node = ControlEspacialNode(kin)
    
    print("\n=======================================================")
    print("      CONTROL DE COORDENADAS ESPACIALES - BRAZO4")
    print("=======================================================")
    print("Ingresa coordenadas cartesianas para mover el robot.")
    print("Escribe 'exit' o 'q' en cualquier momento para salir.")
    print("=======================================================\n")
    
    # Bucle interactivo
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            
            entrada = input("Coordenadas X Y Z (ej. '0.3 0.0 0.2') o Comando: ").strip()
            if not entrada:
                continue
            
            if entrada.lower() in ('exit', 'q'):
                break
                
            partes = entrada.split()
            if len(partes) < 3:
                print("Error: Debes ingresar exactamente 3 valores: X Y Z.")
                continue
                
            try:
                x = float(partes[0])
                y = float(partes[1])
                z = float(partes[2])
                
                node.mover_a_coordenadas(x, y, z)
            except ValueError:
                print("Error: Todos los parámetros de coordenadas deben ser números.")
                
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
