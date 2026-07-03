#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class VisorKinect(Node):
    def __init__(self):
        super().__init__('visor_kinect')
        
        # Suscriptor al estado de las articulaciones
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10
        )
        
        self.q = [0.0, 0.0, 0.0, 0.0]
        self.get_logger().info("Visor Kinect iniciado. Abriendo ventana gráfica independiente...")
        
        # Configuración de la figura interactiva de Matplotlib
        plt.ion()
        self.fig = plt.figure(figsize=(8, 7))
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # Configurar vista desde la perspectiva de la Kinect
        # La Kinect está en X=0.13, Y=0.0, Z=0.015 y mira al frente (+X).
        # Por lo tanto, configuramos la cámara de Matplotlib para mirar en esa dirección (Azim=180, Elev=12)
        self.ax.view_init(elev=12, azim=180)
        
        # Límites del gráfico (Workspace visible al frente del robot)
        self.ax.set_xlim(0.13, 0.75)
        self.ax.set_ylim(-0.4, 0.4)
        self.ax.set_zlim(0.0, 0.6)
        
        self.ax.set_xlabel('X (Frente/Profundidad) - m')
        self.ax.set_ylabel('Y (Lateral) - m')
        self.ax.set_zlabel('Z (Altura) - m')
        self.ax.set_title('Vista en Primera Persona - Cámara Kinect', fontsize=12, fontweight='bold')
        
        # Elementos gráficos iniciales
        self.line_arm, = self.ax.plot([], [], [], 'o-', lw=6, color='#1f77b4', mec='orange', mew=2, label='Brazo Robótico')
        self.line_gripper_left, = self.ax.plot([], [], [], '-', lw=3, color='red', label='Gripper')
        self.line_gripper_right, = self.ax.plot([], [], [], '-', lw=3, color='red')
        
        # Dibujar una representación de la Kinect en la posición X=0.13
        self.ax.scatter([0.13], [0.0], [0.015], color='black', s=150, marker='s', label='Lente Kinect')
        
        self.ax.grid(True)
        self.ax.legend(loc='upper right')
        
    def listener_callback(self, msg):
        q_temp = [0.0, 0.0, 0.0, 0.0]
        for name, pos in zip(msg.name, msg.position):
            if name == 'J1':
                q_temp[0] = pos
            elif name == 'J2':
                q_temp[1] = pos
            elif name == 'J3':
                q_temp[2] = pos
            elif name == 'J4':
                q_temp[3] = pos
        self.q = q_temp

    def get_joint_positions(self):
        q1, q2, q3, q4 = self.q
        
        # Matrices de transformación homogénea
        R_base = np.array([
            [1, 0, 0, -0.68259],
            [0, 1, 0, 1.4348],
            [0, 0, 1, -0.0445],
            [0, 0, 0, 1]
        ])
        
        q1_offset = q1 - 1.3526301702956054
        T_BL_L1 = np.array([
            [np.cos(q1_offset), -np.sin(q1_offset), 0, 0.68259],
            [-np.sin(q1_offset), -np.cos(q1_offset), 0, -1.4348],
            [0, 0, -1, 0.0445],
            [0, 0, 0, 1]
        ])
        
        theta2 = q2 + 0.6425546
        T_L1_L2 = np.array([
            [np.cos(theta2), 0, np.sin(theta2), 0],
            [0, 1, 0, 0],
            [-np.sin(theta2), 0, np.cos(theta2), -0.060218],
            [0, 0, 0, 1]
        ])
        
        theta3 = -q3 + 0.1941317
        T_L2_L3 = np.array([
            [-np.cos(theta3), 0, -np.sin(theta3), -0.16402],
            [0, -1, 0, 0.010151],
            [-np.sin(theta3), 0, np.cos(theta3), -0.42946],
            [0, 0, 0, 1]
        ])
        
        theta4 = -q4 + 1.0730563
        T_L3_L4 = np.array([
            [np.cos(theta4), 0, np.sin(theta4), 0.2336495],
            [0, 1, 0, 0],
            [-np.sin(theta4), 0, np.cos(theta4), -0.0478555],
            [0, 0, 0, 1]
        ])
        
        # Transformaciones
        T1 = R_base @ T_BL_L1
        T2 = T1 @ T_L1_L2
        T3 = T2 @ T_L2_L3
        T4 = T3 @ T_L3_L4
        
        p0 = np.array([0.0, 0.0, 0.0]) # base_link
        p1 = T1[:3, 3]
        p2 = T2[:3, 3]
        p3 = T3[:3, 3]
        p4 = T4[:3, 3]
        
        # Posición de las pinzas del gripper
        # Eje Z de L4 es hacia adelante del efector
        z_dir = T4[:3, 2]
        y_dir = T4[:3, 1]
        
        p_ee = p4 + 0.08 * z_dir
        p_grip_l = p_ee + 0.03 * y_dir
        p_grip_r = p_ee - 0.03 * y_dir
        
        return [p0, p1, p2, p3, p4], p_ee, p_grip_l, p_grip_r

    def update_plot(self):
        pts, p_ee, p_l, p_r = self.get_joint_positions()
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        zs = [p[2] for p in pts]
        
        # Añadir efector final al brazo principal
        xs.append(p_ee[0])
        ys.append(p_ee[1])
        zs.append(p_ee[2])
        
        self.line_arm.set_data(xs, ys)
        self.line_arm.set_3d_properties(zs)
        
        # Dibujar gripper/cuchillas
        self.line_gripper_left.set_data([p_ee[0], p_l[0]], [p_ee[1], p_l[1]])
        self.line_gripper_left.set_3d_properties([p_ee[2], p_l[2]])
        
        self.line_gripper_right.set_data([p_ee[0], p_r[0]], [p_ee[1], p_r[1]])
        self.line_gripper_right.set_3d_properties([p_ee[2], p_r[2]])
        
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    node = VisorKinect()
    
    try:
        while rclpy.ok() and plt.fignum_exists(node.fig.number):
            rclpy.spin_once(node, timeout_sec=0.05)
            node.update_plot()
            plt.pause(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
