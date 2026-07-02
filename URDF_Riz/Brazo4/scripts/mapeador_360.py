#!/usr/bin/env python3
import os
import sys
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory

class RobotKinematicsURDF:
    def __init__(self, urdf_path):
        self.L1, self.L2, self.L3, self.L4 = self._cargar_longitudes_urdf(urdf_path)
        print(f"Longitudes URDF cargadas para Mapeo -> L1: {self.L1:.4f}m, L2: {self.L2:.4f}m, L3: {self.L3:.4f}m, L4: {self.L4:.4f}m")

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
        
        return np.array([q1, q2, q3, q4])

class Mapeador360Node(Node):
    def __init__(self, kinematics):
        super().__init__('mapeador_360')
        self.kin = kinematics
        self.pub_joints = self.create_publisher(JointState, '/joint_commands', 10)
        self.get_logger().info("Nodo 'mapeador_360' iniciado.")

    def ejecutar_barrido_estirado(self, vueltas=1):
        # Ángulos calculados para que el brazo esté 100% recto, estirado y horizontal:
        # En la pose natural de SolidWorks:
        # q2 = 82.5° para alinear el hombro con la horizontal
        # q3 = 86.5° para compensar el giro de codo y dejarlo recto
        # q4 = 6.5° para alinear la pinza
        # Como por defecto esto apunta hacia -X, sumamos 180° (pi) a la base para que apunte hacia adelante (+X)
        q2 = np.radians(82.5)
        q3 = np.radians(86.5)
        q4 = np.radians(6.5)
        
        print(f"\n==============================================================")
        print(f"TEST 1: BARRIDO 360° CON BRAZO 100% RECTO Y HORIZONTAL (+X)")
        print(f"==============================================================")
        print(f"Ángulos fijos para el brazo estirado:")
        print(f"  J2={np.degrees(q2):.2f}° | J3={np.degrees(q3):.2f}° | J4={np.degrees(q4):.2f}°")
        print(f"Alcance horizontal total: {self.kin.L2 + self.kin.L3 + self.kin.L4:.4f} metros")
        print("Comenzando en 3 segundos... ¡Mira RViz!")
        time.sleep(3)
        
        pasos = np.arange(0, 360 * vueltas + 1, 2)
        for deg in pasos:
            # Ángulo de giro + pi (180 deg de desfase para iniciar mirando adelante en +X)
            rad_j1 = np.radians(deg) + np.pi
            rad_j1_normalized = np.arctan2(np.sin(rad_j1), np.cos(rad_j1))
            
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = ['J1', 'J2', 'J3', 'J4']
            msg.position = [float(rad_j1_normalized), float(q2), float(q3), float(q4)]
            self.pub_joints.publish(msg)
            
            sys.stdout.write(f"\rGirando J1: {deg % 360:3.0f}° / 360° | Vuelta: {int(deg // 360) + 1}/{vueltas}")
            sys.stdout.flush()
            time.sleep(0.04)
        print("\n[Éxito] Test 1 finalizado.")

    def ejecutar_barrido_ik(self, radio=0.78, altura=0.06, phi=0.0, vueltas=1):
        print(f"\n==============================================================")
        print(f"TEST 2: BARRIDO 360° CON CINEMÁTICA INVERSA AL ALCANCE MÁXIMO")
        print(f"==============================================================")
        print(f"  -> Radio del círculo: {radio} m (Alcance máximo = {self.kin.L2 + self.kin.L3 + self.kin.L4:.4f}m)")
        print(f"  -> Altura (Z): {altura} m")
        print(f"  -> Orientación Pinza (Phi): {phi} grados")
        
        try:
            # Calculamos para Y=0, X=radio (ángulo de base 0)
            q_start = self.kin.cinematica_inversa(radio, 0.0, altura, phi)
            q2, q3, q4 = q_start[1], q_start[2], q_start[3]
            print(f"Configuración del brazo calculada:")
            print(f"  J2={np.degrees(q2):.2f}°, J3={np.degrees(q3):.2f}°, J4={np.degrees(q4):.2f}°")
        except ValueError:
            print("[Error] Las coordenadas solicitadas están fuera de los límites geométricos.")
            return

        print("Comenzando en 3 segundos... ¡Mira RViz!")
        time.sleep(3)
        
        pasos = np.arange(0, 360 * vueltas + 1, 2)
        for deg in pasos:
            rad_j1 = np.radians(deg)
            rad_j1_normalized = np.arctan2(np.sin(rad_j1), np.cos(rad_j1))
            
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = ['J1', 'J2', 'J3', 'J4']
            msg.position = [float(rad_j1_normalized), float(q2), float(q3), float(q4)]
            self.pub_joints.publish(msg)
            
            sys.stdout.write(f"\rGirando base: {deg % 360:3.0f}° / 360° | Vuelta: {int(deg // 360) + 1}/{vueltas}")
            sys.stdout.flush()
            time.sleep(0.04)
        print("\n[Éxito] Test 2 finalizado.")

def main():
    try:
        pkg_share = get_package_share_directory('brazo4')
        urdf_path = os.path.join(pkg_share, 'urdf', 'brazo4central.urdf')
    except Exception:
        urdf_path = '/home/joel/Descargas/Brazo4/urdf/brazo4central.urdf'
        
    kin = RobotKinematicsURDF(urdf_path)
    
    rclpy.init()
    node = Mapeador360Node(kin)
    
    try:
        # Ejecutar Test 1: Brazo 100% estirado directamente en X/Y horizontal
        node.ejecutar_barrido_estirado(vueltas=1)
        
        time.sleep(2)
        
        # Ejecutar Test 2: Brazo en extensión máxima resolviendo cinemática inversa (R = 0.78 m, Z = 0.06 m, Phi = 0 deg)
        node.ejecutar_barrido_ik(radio=0.78, altura=0.06, phi=0.0, vueltas=1)
        
    except KeyboardInterrupt:
        print("\nMapeo interrumpido por el usuario.")
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
