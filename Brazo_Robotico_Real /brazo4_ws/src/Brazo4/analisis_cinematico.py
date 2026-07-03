#!/usr/bin/env python3
import numpy as np
import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os

class RobotKinematicsURDF:
    def __init__(self, urdf_path):
        self.L1, self.L2, self.L3, self.L4 = self._cargar_longitudes_urdf(urdf_path)
        print(f"Longitudes calculadas desde URDF -> L1:{self.L1:.4f}m, L2:{self.L2:.4f}m, L3:{self.L3:.4f}m, L4:{self.L4:.4f}m")

    def _cargar_longitudes_urdf(self, urdf_path):
        # Parsea el archivo URDF para extraer las distancias entre joints
        try:
            root = ET.parse(urdf_path).getroot()
        except Exception as e:
            print(f"Error cargando URDF {urdf_path}: {e}")
            # Valores por defecto en metros
            return 0.06022, 0.45972, 0.25850, 0.08000

        # Diccionario para guardar las posiciones relativas de cada articulación
        origins = {}
        for joint in root.findall('joint'):
            name = joint.get('name')
            origin = joint.find('origin')
            if origin is not None:
                xyz = [float(x) for x in origin.get('xyz', '0 0 0').split()]
                origins[name] = xyz

        try:
            # En el URDF de Brazo4, las articulaciones se llaman J1, J2, J3, J4
            # J2 está offseteado en Z relativo a J1 (L1)
            l1 = origins['J2'][2]  # Desplazamiento Z de L1 a L2 (es -0.060218)
            # J3 está offseteado relativo a J2 (L2)
            l2 = np.linalg.norm(origins['J3'])  # Distancia de J2 a J3 (0.45972 m)
            # J4 está offseteado relativo a J3 (L3)
            l3 = np.linalg.norm(origins['J4'])  # Distancia de J3 a J4 (0.25850 m)
            # Para L4, usamos la distancia del gripper_base al extremo del efector (80 mm por defecto)
            l4 = 0.08000  # metros (80 mm) por defecto para el efector final
        except KeyError as e:
            print(f"Advertencia: No se encontraron los joints esperados en el URDF. Usando valores por defecto. Falta: {e}")
            # Valores por defecto en metros
            return 0.06022, 0.45972, 0.25850, 0.08000
            
        return abs(l1), abs(l2), abs(l3), abs(l4)

    def cinematica_directa(self, q1, q2, q3, q4):
        theta23 = q2 + q3
        theta234 = q2 + q3 + q4

        x = np.cos(q1) * (self.L2 * np.cos(q2) + self.L3 * np.cos(theta23) + self.L4 * np.cos(theta234))
        y = np.sin(q1) * (self.L2 * np.cos(q2) + self.L3 * np.cos(theta23) + self.L4 * np.cos(theta234))
        z = self.L1 + self.L2 * np.sin(q2) + self.L3 * np.sin(theta23) + self.L4 * np.sin(theta234)
        
        return np.array([x, y, z])

    def cinematica_inversa(self, x, y, z, phi_deg):
        phi = np.radians(phi_deg)
        
        # 1. Ángulo de la base
        q1 = np.arctan2(y, x)
        
        # 2. Proyección en plano 2D
        R = np.sqrt(x**2 + y**2)
        R_prima = R - self.L4 * np.cos(phi)
        Z_prima = z - self.L1 - self.L4 * np.sin(phi)
        
        # 3. Teorema del Coseno para q3
        num = (R_prima**2) + (Z_prima**2) - (self.L2**2) - (self.L3**2)
        den = 2 * self.L2 * self.L3
        D = num / den
        
        if abs(D) > 1.0:
            raise ValueError("El punto objetivo está fuera del espacio de trabajo del robot.")
            
        q3 = np.arctan2(np.sqrt(max(0.0, 1.0 - D**2)), D)  # Solución codo arriba
        
        # 4. Cálculo de q2
        q2 = np.arctan2(Z_prima, R_prima) - np.arctan2(self.L3 * np.sin(q3), self.L2 + self.L3 * np.cos(q3))
        
        # 5. Cálculo de q4
        q4 = phi - q2 - q3
        
        return np.array([q1, q2, q3, q4])

def run_simulation(num_samples=5000):
    urdf_path = '/home/joel/Descargas/Brazo4/urdf/brazo4central.urdf'
    print(f"Cargando URDF desde: {urdf_path}")
    
    robot = RobotKinematicsURDF(urdf_path)
    
    print(f"Generando simulación de Monte Carlo con {num_samples} muestras...")
    
    # Límites de los joints
    limits = [
        (-np.pi, np.pi),       # J1
        (-np.pi/2, np.pi/2),   # J2
        (-np.pi/2, np.pi/2),   # J3
        (-np.pi/2, np.pi/2)    # J4
    ]
    
    xs, ys, zs = [], [], []
    
    for _ in range(num_samples):
        # Generar ángulos aleatorios dentro de los límites
        q1 = np.random.uniform(limits[0][0], limits[0][1])
        q2 = np.random.uniform(limits[1][0], limits[1][1])
        q3 = np.random.uniform(limits[2][0], limits[2][1])
        q4 = np.random.uniform(limits[3][0], limits[3][1])
        
        pos = robot.cinematica_directa(q1, q2, q3, q4)
        
        xs.append(pos[0])
        ys.append(pos[1])
        zs.append(pos[2])
        
    # Crear la gráfica
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # Graficar con color según la coordenada Z
    sc = ax.scatter(xs, ys, zs, c=zs, cmap='viridis', s=2, alpha=0.6)
    
    ax.set_title('Workspace del Brazo4 (Ecuaciones Analíticas)', fontsize=14)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    
    cbar = plt.colorbar(sc, ax=ax, pad=0.1)
    cbar.set_label('Coordenada Z (m)', rotation=270, labelpad=15)
    
    # Guardar la gráfica en la carpeta de documentos
    output_image = '/home/joel/Descargas/Brazo4/workspace_brazo4.png'
    plt.savefig(output_image, dpi=300, bbox_inches='tight')
    print(f"Gráfica guardada exitosamente en: {output_image}")
    
    # Estadísticas del Workspace
    xs = np.array(xs)
    ys = np.array(ys)
    zs = np.array(zs)
    radii = np.sqrt(xs**2 + ys**2 + zs**2)
    
    print("\n=== RESULTADOS DEL ANÁLISIS CINEMÁTICO ===")
    print(f"Alcance máximo radial: {np.max(radii):.3f} m")
    print(f"Alcance mínimo radial: {np.min(radii):.3f} m")
    print(f"Límites en X: [{np.min(xs):.3f}, {np.max(xs):.3f}] m")
    print(f"Límites en Y: [{np.min(ys):.3f}, {np.max(ys):.3f}] m")
    print(f"Límites en Z: [{np.min(zs):.3f}, {np.max(zs):.3f}] m")
    
if __name__ == '__main__':
    run_simulation()
