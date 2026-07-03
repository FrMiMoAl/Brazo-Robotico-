#!/usr/bin/env python3
import sys
import numpy as np

def kabsch_calibration(points_camera, points_base):
    """
    Calcula la transformacion rigida base = R * camera + t
    usando el algoritmo de Kabsch.
    
    points_camera: array de numpy de forma (N, 3)
    points_base: array de numpy de forma (N, 3)
    """
    # 1. Calcular centroides
    centroid_camera = np.mean(points_camera, axis=0)
    centroid_base = np.mean(points_base, axis=0)
    
    # 2. Centrar los puntos
    c_camera = points_camera - centroid_camera
    c_base = points_base - centroid_base
    
    # 3. Matriz de covarianza
    H = np.dot(c_camera.T, c_base)
    
    # 4. SVD de H
    U, S, Vt = np.linalg.svd(H)
    
    # 5. Rotacion R
    R = np.dot(Vt.T, U.T)
    
    # Manejar caso de reflexion
    if np.linalg.det(R) < 0:
        print("[INFO] Corrigiendo reflexion detectada en la matriz de rotacion...")
        Vt[2, :] *= -1
        R = np.dot(Vt.T, U.T)
        
    # 6. Traslacion t
    t = centroid_base - np.dot(R, centroid_camera)
    
    return R, t

def rotation_matrix_to_euler(R):
    """
    Convierte una matriz de rotacion a angulos de Euler (Roll, Pitch, Yaw)
    siguiendo la convencion de ROS (X-Y-Z fixed / Z-Y-X intrinsic).
    """
    # R31 = -sin(pitch)
    pitch = np.arctan2(-R[2, 0], np.sqrt(R[0, 0]**2 + R[1, 0]**2))
    
    # Si pitch no esta en gimbal lock
    if np.abs(pitch - np.pi/2) > 1e-5 and np.abs(pitch + np.pi/2) > 1e-5:
        roll = np.arctan2(R[2, 1], R[2, 2])
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        # Gimbal lock
        roll = 0.0
        yaw = np.arctan2(-R[0, 1], R[1, 1])
        
    return roll, pitch, yaw

def main():
    print("=========================================================")
    print("    CALIBRADOR CÁMARA-BRAZO (ALGORITMO DE KABSCH)        ")
    print("=========================================================\n")
    print("Por favor, ingresa al menos 4 puntos en el frame de la CÁMARA")
    print("y sus correspondientes posiciones medidas físicamente en la BASE.\n")

    # -----------------------------------------------
    # 1) Intentar cargar un archivo de muestras
    # -----------------------------------------------
    import os
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--samples_file', help="Archivo con puntos (Xb Yb Zb Xc Yc Zc)", default=None)
    args, unknown = parser.parse_known_args()
    samples_file = args.samples_file

    if samples_file and os.path.isfile(samples_file):
        print(f"[INFO] Cargando muestras desde archivo: {samples_file}")
        data = np.loadtxt(samples_file)
        if data.shape[1] != 6:
            print("[ERROR] El archivo debe contener 6 columnas (X_base Y_base Z_base X_cam Y_cam Z_cam).")
            sys.exit(1)
        if data.shape[0] < 4:
            print("[ERROR] Se requieren al menos 4 puntos para calibrar.")
            sys.exit(1)
        # Separar en matrices
        points_base = data[:, 0:3]
        points_cam = data[:, 3:6]
    else:
        # ------------------------------------------------
        # 2) Modo interactivo
        # ------------------------------------------------
        try:
            n_points = int(input("¿Cuántos puntos de calibración vas a ingresar? (mínimo 4): "))
            if n_points < 4:
                print("[ERROR] Se necesitan al menos 4 puntos para una calibración estable.")
                sys.exit(1)
        except ValueError:
            print("[ERROR] Entrada inválida. Usando ejemplo predeterminado de 5 puntos para validación.")
            n_points = 5

        points_cam_list = []
        points_base_list = []

        print("\n--- INGRESO DE COORDENADAS (Unidades en METROS) ---")
        for i in range(n_points):
            print(f"\n[Punto {i+1}]")
            # Base
            while True:
                try:
                    inp = input("  Posición del objeto en BASE (X Y Z respecto a base_link): ")
                    if not inp.strip():
                        raise ValueError
                    x, y, z = map(float, inp.strip().split())
                    points_base_list.append([x, y, z])
                    break
                except ValueError:
                    print("  [ERROR] Formato inválido. Ingresá 3 números flotantes separados por espacio (Ej: 0.20 0.0 0.05)")
            # Cámara
            while True:
                try:
                    inp = input("  Lectura en CÁMARA /perception/selected_object_camera (X Y Z): ")
                    if not inp.strip():
                        raise ValueError
                    x, y, z = map(float, inp.strip().split())
                    points_cam_list.append([x, y, z])
                    break
                except ValueError:
                    print("  [ERROR] Formato inválido. Ingresá 3 números flotantes separados por espacio (Ej: 0.03 -0.02 0.45)")

        points_base = np.array(points_base_list)
        points_cam = np.array(points_cam_list)

    # Ejecutar Kabsch
    R, t = kabsch_calibration(points_cam, points_base)
    roll, pitch, yaw = rotation_matrix_to_euler(R)

    # Calcular error de alineacion medio (RMSD)
    aligned_cam = np.dot(points_cam, R.T) + t
    errors = np.linalg.norm(aligned_cam - points_base, axis=1)
    rmsd = np.sqrt(np.mean(errors**2))
    
    print("\n=========================================================")
    print("                  RESULTADOS DE CALIBRACIÓN              ")
    print("=========================================================")
    print(f"Error medio de alineación (RMSD): {rmsd*100.0:.2f} cm")
    if rmsd > 0.03:
        print("[WARNING] El error es mayor a 3 cm. Verificá si las mediciones físicas son correctas.")
    else:
        print("[OK] Calibración de excelente calidad (error < 3 cm).")
        
    print("\n--- Valores de Transformación (ROS 2) ---")
    print(f"Traslación X : {t[0]:.4f} m")
    print(f"Traslación Y : {t[1]:.4f} m")
    print(f"Traslación Z : {t[2]:.4f} m")
    print(f"Rotación Roll  : {roll:.4f} rad ({np.degrees(roll):.1f}°)")
    print(f"Rotación Pitch : {pitch:.4f} rad ({np.degrees(pitch):.1f}°)")
    print(f"Rotación Yaw   : {yaw:.4f} rad ({np.degrees(yaw):.1f}°)")
    
    print("\n=========================================================")
    print("        COMANDO ROS 2 LISTO PARA COPIAR Y EJECUTAR       ")
    print("=========================================================")
    cmd = (f"ros2 run tf2_ros static_transform_publisher \\\n"
           f"  {t[0]:.4f} {t[1]:.4f} {t[2]:.4f} \\\n"
           f"  {roll:.4f} {pitch:.4f} {yaw:.4f} \\\n"
           f"  base_link kinect2_depth_optical_frame")
    print(cmd)
    print("=========================================================\n")

if __name__ == "__main__":
    main()
