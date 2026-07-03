#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import numpy as np

from std_msgs.msg import Float32, Int32
from geometry_msgs.msg import Point 

class KinematicsControlNode(Node):
    def __init__(self):
        super().__init__('kinematics_control_node')
        
        # 1. PUBLICADORES (Hacia los tópicos del ESP32)
        self.pub_pololu = self.create_publisher(Float32, '/motor_pololu/target_deg', 10)
        self.pub_nema = self.create_publisher(Float32, '/motor_nema/target_deg', 10)
        self.pub_servo1 = self.create_publisher(Int32, '/servo1/target_deg', 10)
        self.pub_servo2 = self.create_publisher(Int32, '/servo2/target_deg', 10)
        
        # 2. SUSCRIPTOR
        self.sub_coord = self.create_subscription(Point, '/brazo/target_xyz', self.listener_callback, 10)
        
        # --- ESTADO INTERNO ---
        self.q_current = np.array([0.0, np.radians(-30.0), np.radians(30.0), np.radians(90.0)])
        
        self.joint_limits = [
            (-np.pi, np.pi),
            (-np.pi, np.pi),
            (-np.pi, np.pi),
            (-np.pi, np.pi)
        ]

        # Origen corregido en base al NEMA
        self.joints_data = [
            {'xyz': [0.0, 0.0, 0.042792], 'rpy': [0.0, 0.0, 0.0], 'axis': [0.0, 0.0, 1.0]},
            {'xyz': [0.0, 0.0, 0.093058], 'rpy': [1.542194, -1.047020, 2.119165], 'axis': [0.500000, -0.866025, 0.000000]},
            {'xyz': [0.118563, 0.068452, 0.376872], 'rpy': [0.000000, 1.150442, 0.523599], 'axis': [0.000000, -1.000000, 0.000000]},
            {'xyz': [0.23364969, 0.000000, -0.04785507], 'rpy': [0.000000, -0.359292, 0.000000], 'axis': [0.000000, -1.000000, 0.000000]}
        ]
        
        self.get_logger().info("Nodo de Cinemática Inversa activo con corrección de desfase superior.")

    def _rot_axis(self, axis, theta):
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

    def _rpy_matrix(self, r, p, y):
        cR, sR = np.cos(r), np.sin(r)
        cP, sP = np.cos(p), np.sin(p)
        cY, sY = np.cos(y), np.sin(y)
        Rx = np.array([[1, 0, 0], [0, cR, -sR], [0, sR, cR]])
        Ry = np.array([[cP, 0, sP], [0, 1, 0], [-sP, 0, cP]])
        Rz = np.array([[cY, -sY, 0], [sY, cY, 0], [0, 0, 1]])
        return Rz @ Ry @ Rx

    def _joint_transform(self, xyz, rpy, axis, q):
        T = np.eye(4)
        T[:3, 3] = xyz
        T[:3, :3] = self._rpy_matrix(*rpy) @ self._rot_axis(axis, q)
        return T

    def forward_kinematics_ee(self, q):
        T = np.eye(4)
        for i in range(4):
            jd = self.joints_data[i]
            T_joint = self._joint_transform(jd['xyz'], jd['rpy'], jd['axis'], q[i])
            T = T @ T_joint
        T_tip = np.eye(4)
        T_tip[0, 3] = 0.08
        T = T @ T_tip
        return T[:3, 3]

    def inverse_kinematics(self, target_pos, max_iters=200, tol=1e-3):
        q = self.q_current.copy()
        damping = 0.01
        for i in range(max_iters):
            q[0] = np.arctan2(np.sin(q[0]), np.cos(q[0]))
            pos = self.forward_kinematics_ee(q)
            error = target_pos - pos
            if np.linalg.norm(error) < tol:
                q[0] = np.arctan2(np.sin(q[0]), np.cos(q[0]))
                return q, True
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
            for j in range(1, 4):
                low, high = self.joint_limits[j]
                q[j] = np.clip(q[j], low, high)
        q[0] = np.arctan2(np.sin(q[0]), np.cos(q[0]))
        pos_final = self.forward_kinematics_ee(q)
        if np.linalg.norm(target_pos - pos_final) < 0.02:
            return q, True
        return q, False

    def listener_callback(self, msg):
        x = msg.x
        y = msg.y
        z = msg.z
        
        self.get_logger().info(f"Recibida coordenada objetivo -> X: {x}, Y: {y}, Z: {z}")
        
        target_pos = np.array([x, y, z])
        q_sol, success = self.inverse_kinematics(target_pos)
        
        if not success:
            self.get_logger().error("¡ERROR: Coordenada fuera del alcance físico!")
            return
            
        self.q_current = q_sol.copy()
        q_sol[0] = np.arctan2(np.sin(q_sol[0]), np.cos(q_sol[0]))
        
        j1_raw = math.degrees(q_sol[0])
        j2_raw = math.degrees(q_sol[1])
        j3_raw = math.degrees(q_sol[2])
        j4_raw = math.degrees(q_sol[3])

        # --- ASIGNACIÓN CON AJUSTES DE COMPENSACIÓN GEOMÉTRICA ---
        q1_deg = j1_raw * -1.0                  
        
        # COMPENSACIÓN POLOLU (Hombro): Le restamos un offset fino (ej. -15 grados) 
        # para obligar mecánicamente al hombro a tirar hacia atrás y corregir la inclinación frontal.
        q2_deg = (j2_raw * 2.0) - 15.0                  
        
        q3_deg = j4_raw       # Servo 1 (j4)
        
        # COMPENSACIÓN SERVO 2 (Codo): Le sumamos un pequeño offset (ej. +12 grados)
        # para que abra un poco más el codo y mantenga la pinza apuntando recto arriba.
        q4_deg = j3_raw + 12.0       # Servo 2 (j3)

        # --- VALIDACIÓN DE LÍMITES ---
        if not (-320.0 <= q2_deg <= 0.0):
            self.get_logger().warn(f"Límite Pololu excedido: {q2_deg:.2f}°")
            return
        if not (0.0 <= q1_deg <= 130.0):
            self.get_logger().warn(f"Límite NEMA excedido: {q1_deg:.2f}°")
            return
        if not (0.0 <= q3_deg <= 180.0):
            self.get_logger().warn(f"Límite Servo 1 excedido: {q3_deg:.2f}°")
            return
        if not (0.0 <= q4_deg <= 150.0):
            self.get_logger().warn(f"Límite Servo 2 excedido: {q4_deg:.2f}°")
            return

        # --- PUBLICACIÓN DE COMANDOS ---
        msg_pololu = Float32(data=float(q2_deg))
        msg_nema = Float32(data=float(q1_deg))
        msg_servo1 = Int32(data=int(round(q3_deg)))
        msg_servo2 = Int32(data=int(round(q4_deg)))
        
        self.pub_pololu.publish(msg_pololu)
        self.pub_nema.publish(msg_nema)
        self.pub_servo1.publish(msg_servo1)
        self.pub_servo2.publish(msg_servo2)
        
        self.get_logger().info(f"Movimiento enviado -> Pololu: {q2_deg:.2f}°, NEMA: {q1_deg:.2f}°, Servo1: {q3_deg:.2f}°, Servo2: {q4_deg:.2f}°")

def main(args=None):
    rclpy.init(args=args)
    node = KinematicsControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()