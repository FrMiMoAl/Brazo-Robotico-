#!/usr/bin/env python3
import os
import sys
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import numpy as np
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory

class RobotKinematicsURDF:
    def __init__(self, urdf_path):
        self.L1, self.L2, self.L3, self.L4 = self._cargar_longitudes_urdf(urdf_path)

    def _cargar_longitudes_urdf(self, urdf_path):
        try:
            root = ET.parse(urdf_path).getroot()
        except Exception as e:
            return 0.06022, 0.45972, 0.25850, 0.08000

        origins = {}
        for joint in root.findall('joint'):
            name = joint.get('name')
            origin = joint.find('origin')
            if origin is not None:
                xyz = [float(x) for x in origin.get('xyz', '0 0 0').split()]
                origins[name] = xyz

        try:
            l1 = origins['J2'][2]
            l2 = np.linalg.norm(origins['J3'])
            l3 = np.linalg.norm(origins['J4'])
            l4 = 0.08000
        except KeyError:
            return 0.06022, 0.45972, 0.25850, 0.08000
            
        return abs(l1), abs(l2), abs(l3), abs(l4)

    def cinematica_inversa(self, x, y, z, phi_deg):
        phi = np.radians(phi_deg)
        phi = np.arctan2(np.sin(phi), np.cos(phi))
        
        q1 = np.arctan2(y, x)
        
        R = np.sqrt(x**2 + y**2)
        R_prima = R - self.L4 * np.cos(phi)
        Z_prima = z - self.L1 - self.L4 * np.sin(phi)
        
        num = (R_prima**2) + (Z_prima**2) - (self.L2**2) - (self.L3**2)
        den = 2 * self.L2 * self.L3
        D = num / den
        
        if abs(D) > 1.0:
            raise ValueError("Fuera de alcance")
            
        q3 = np.arctan2(np.sqrt(max(0.0, 1.0 - D**2)), D)
        q2 = np.arctan2(Z_prima, R_prima) - np.arctan2(self.L3 * np.sin(q3), self.L2 + self.L3 * np.cos(q3))
        q4 = phi - q2 - q3
        q4 = np.arctan2(np.sin(q4), np.cos(q4))
        
        # Compensar offsets del URDF
        q2_cmd = q2 + 0.23011
        q3_cmd = q3 + 0.504
        q4_cmd = q4 + 0.49774
        
        return np.array([q1, q2_cmd, q3_cmd, q4_cmd])

class MapeadorEspacio(Node):
    def __init__(self, kinematics):
        super().__init__('mapeador_espacio')
        self.kin = kinematics
        self.pub_joints = self.create_publisher(JointState, '/joint_commands', 10)
        self.pub_grip = self.create_publisher(Bool, '/gripper_command', 10)
        
        # Límites para filtrado (los mismos que control_brazo.py)
        self.joint_limits = [
            (-np.pi, np.pi),                          # J1
            (np.radians(-100), np.radians(40)),       # J2 (-100 a 40)
            (np.radians(-240), np.radians(40)),       # J3 (-240 a 40)
            (np.radians(-90), np.radians(90))         # J4 (-90 a 90)
        ]
        
        self.trajectory = self.generar_trayectoria_mapeo()
        self.get_logger().info(f'Trayectoria de mapeo generada con {len(self.trajectory)} puntos alcanzables.')
        
        # Timer para ejecutar la trayectoria
        self.index = 0
        self.timer = self.create_timer(0.08, self.timer_callback) # 80ms por paso para movimiento fluido

    def generar_trayectoria_mapeo(self):
        trajectory = []
        
        # Parámetros del barrido cilíndrico
        thetas = np.linspace(-np.pi, np.pi, 36) # 360 grados en pasos de 10 grados
        radii = np.linspace(0.30, 0.70, 9)     # Alcance radial de 30cm a 70cm
        heights = np.linspace(0.0, 0.60, 9)    # Altura Z de 0cm a 60cm
        
        # Ángulos de pitch preferidos a probar para cada punto
        pitch_options = [0.0, -15.0, -30.0, -45.0, 15.0, 30.0, 45.0]
        
        for i, theta in enumerate(thetas):
            # Hacer barrido de radio alternando dirección para suavidad
            r_list = radii if i % 2 == 0 else reversed(radii)
            for j, r in enumerate(r_list):
                # Hacer barrido de altura alternando dirección para suavidad
                z_list = heights if j % 2 == 0 else reversed(heights)
                for z in z_list:
                    x = r * np.cos(theta)
                    y = r * np.sin(theta)
                    
                    # Probar las opciones de orientación hasta encontrar una alcanzable dentro de límites
                    for phi in pitch_options:
                        try:
                            q = self.kin.cinematica_inversa(x, y, z, phi)
                            
                            # Verificar límites de juntas
                            valid = True
                            for idx in range(4):
                                low, high = self.joint_limits[idx]
                                if not (low <= q[idx] <= high):
                                    valid = False
                                    break
                            
                            if valid:
                                trajectory.append(q)
                                break # Encontró solución para este punto, pasar al siguiente
                        except ValueError:
                            continue
        return trajectory

    def timer_callback(self):
        if self.index >= len(self.trajectory):
            self.get_logger().info('¡Mapeo del espacio de trabajo completado con éxito!')
            # Volver a empezar en bucle continuo
            self.index = 0
            
        q = self.trajectory[self.index]
        
        # Publicar articulaciones
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['J1', 'J2', 'J3', 'J4']
        msg.position = [float(val) for val in q]
        self.pub_joints.publish(msg)
        
        # Hacer que el gripper se abra y cierre rítmicamente para verificar funcionamiento
        if self.index % 25 == 0:
            grip_msg = Bool()
            # Alternar estado del gripper
            grip_msg.data = (self.index % 50 == 0)
            self.pub_grip.publish(grip_msg)
            
        self.index += 1

def main():
    # Encontrar URDF
    urdf_path = ""
    try:
        share_dir = get_package_share_directory('brazo4')
        urdf_path = os.path.join(share_dir, 'urdf', 'brazo4central.urdf')
    except Exception:
        # Fallback local
        urdf_path = "/home/joel/Descargas/Brazo4/urdf/brazo4central.urdf"
        
    kin = RobotKinematicsURDF(urdf_path)
    
    rclpy.init()
    node = MapeadorEspacio(kin)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
