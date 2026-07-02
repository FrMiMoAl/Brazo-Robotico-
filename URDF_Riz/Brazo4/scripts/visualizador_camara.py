#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import cv2

class VisualizadorCamara(Node):
    def __init__(self):
        super().__init__('visualizador_camara')
        
        # Suscriptor al estado de las articulaciones
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10
        )
        
        self.q = [0.0, 0.0, 0.0, 0.0]
        self.get_logger().info("Visualizador de Cámara OpenCV iniciado. Abriendo ventana...")
        
        # Parámetros ópticos de la cámara (Kinect)
        self.width = 640
        self.height = 480
        self.f = 220.0  # Reducido de 450 a 220 para dar un FOV gran angular y ver todo el robot
        self.cx = 320.0
        self.cy = 240.0
        
        # Posición de la Kinect: X=0.13, Y=0.0, Z=0.015 (en el frame de base_link)
        self.cam_x = 0.13
        self.cam_y = 0.0
        self.cam_z = 0.015

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

    def to_cam_frame(self, p):
        """
        Transforma un punto [x, y, z] en base_link al frame óptico de la cámara.
        Z_c apunta adelante (+X robot), X_c apunta a la derecha (-Y robot), Y_c apunta abajo (-Z robot).
        """
        x, y, z = p
        zc = x - self.cam_x
        xc = -y
        yc = -z + self.cam_z
        return np.array([xc, yc, zc])

    def project_point(self, p):
        """
        Proyecta un punto 3D [x, y, z] en el frame base_link al plano de imagen 2D.
        Retorna (u, v) o None si está detrás del plano de recorte cercano.
        """
        c = self.to_cam_frame(p)
        if c[2] <= 0.02:  # Plano cercano a 2 cm
            return None
        u = int(self.cx + self.f * c[0] / c[2])
        v = int(self.cy + self.f * c[1] / c[2])
        return (u, v)

    def project_line(self, pA, pB):
        """
        Proyecta un segmento de línea entre pA y pB en 3D al plano 2D,
        aplicando recorte matemático (clipping) si uno de los puntos está detrás de la cámara (zc <= 0.02).
        Retorna ((uA, vA), (uB, vB)) o None si todo el segmento está detrás.
        """
        cA = self.to_cam_frame(pA)
        cB = self.to_cam_frame(pB)
        
        near = 0.02
        
        # Ambos detrás
        if cA[2] < near and cB[2] < near:
            return None
            
        # Recortar cA si está detrás
        if cA[2] < near:
            t = (near - cA[2]) / (cB[2] - cA[2])
            cA = cA + t * (cB - cA)
            
        # Recortar cB si está detrás
        if cB[2] < near:
            t = (near - cB[2]) / (cA[2] - cB[2])
            cB = cB + t * (cA - cB)
            
        uA = int(self.cx + self.f * cA[0] / cA[2])
        vA = int(self.cy + self.f * cA[1] / cA[2])
        
        uB = int(self.cx + self.f * cB[0] / cB[2])
        vB = int(self.cy + self.f * cB[1] / cB[2])
        
        return (uA, vA), (uB, vB)

    def get_joint_positions(self):
        q1, q2, q3, q4 = self.q
        
        # Matrices de transformación homogénea (cinemática directa)
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
        
        T1 = R_base @ T_BL_L1
        T2 = T1 @ T_L1_L2
        T3 = T2 @ T_L2_L3
        T4 = T3 @ T_L3_L4
        
        p0 = np.array([0.0, 0.0, 0.0]) # base_link
        p1 = T1[:3, 3]
        p2 = T2[:3, 3]
        p3 = T3[:3, 3]
        p4 = T4[:3, 3]
        
        # Orientación del efector final
        z_dir = T4[:3, 2]
        y_dir = T4[:3, 1]
        
        p_ee = p4 + 0.08 * z_dir
        p_grip_l = p_ee + 0.03 * y_dir
        p_grip_r = p_ee - 0.03 * y_dir
        
        return [p0, p1, p2, p3, p4], p_ee, p_grip_l, p_grip_r

    def draw_scene(self):
        # Crear lienzo vacío (Fondo gris oscuro degradado)
        canvas = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        for y in range(self.height):
            val = int(25 + 30 * (y / self.height))
            canvas[y, :] = [val, val, val]
            
        # 1. Dibujar el plano del suelo (Grid de profundidad)
        grid_color = (95, 95, 95) # Gris más brillante para visibilidad
        
        # Líneas a lo largo de X
        for y_g in np.arange(-0.5, 0.6, 0.1):
            pts_line = []
            for x_g in np.arange(0.15, 1.2, 0.05):
                proj = self.project_point([x_g, y_g, 0.0])
                if proj:
                    pts_line.append(proj)
            for i in range(len(pts_line) - 1):
                cv2.line(canvas, pts_line[i], pts_line[i+1], grid_color, 1)

        # Líneas a lo largo de Y
        for x_g in np.arange(0.15, 1.2, 0.1):
            pts_line = []
            for y_g in np.arange(-0.5, 0.55, 0.05):
                proj = self.project_point([x_g, y_g, 0.0])
                if proj:
                    pts_line.append(proj)
            for i in range(len(pts_line) - 1):
                cv2.line(canvas, pts_line[i], pts_line[i+1], grid_color, 1)

        # 2. Obtener posiciones de articulaciones
        pts, p_ee, p_l, p_r = self.get_joint_positions()
        
        # Proyectar todas las articulaciones individuales (para dibujar círculos de juntas)
        projs = [self.project_point(p) for p in pts]
        proj_ee = self.project_point(p_ee)
        
        # 3. Dibujar la Base (Si es visible)
        base_proj = projs[0]
        if base_proj:
            cv2.circle(canvas, base_proj, 25, (80, 80, 80), -1)
            cv2.circle(canvas, base_proj, 25, (160, 160, 160), 2)
            
        # 4. Dibujar Eslabones con recorte matemático (clipping) para evitar que desaparezcan
        # Eslabón 1: J1 -> J2
        line1 = self.project_line(pts[1], pts[2])
        if line1:
            cv2.line(canvas, line1[0], line1[1], (180, 130, 0), 14) # Dorado oscuro
            cv2.line(canvas, line1[0], line1[1], (255, 190, 50), 6)  # Resplandor interno
        
        # Eslabón 2: J2 -> J3
        line2 = self.project_line(pts[2], pts[3])
        if line2:
            cv2.line(canvas, line2[0], line2[1], (150, 0, 150), 10) # Púrpura
            cv2.line(canvas, line2[0], line2[1], (230, 80, 230), 4)
            
        # Eslabón 3: J3 -> J4
        line3 = self.project_line(pts[3], pts[4])
        if line3:
            cv2.line(canvas, line3[0], line3[1], (0, 130, 0), 8) # Verde
            cv2.line(canvas, line3[0], line3[1], (80, 220, 80), 3)

        # Eslabón 4: J4 -> EE
        line4 = self.project_line(pts[4], p_ee)
        if line4:
            cv2.line(canvas, line4[0], line4[1], (0, 80, 220), 6) # Azul
            cv2.line(canvas, line4[0], line4[1], (80, 150, 255), 2)

        # 5. Dibujar Gripper (Cuchillas de la tijera)
        line_grip_l = self.project_line(p_ee, p_l)
        line_grip_r = self.project_line(p_ee, p_r)
        if line_grip_l:
            cv2.line(canvas, line_grip_l[0], line_grip_l[1], (0, 0, 255), 4)
            cv2.circle(canvas, line_grip_l[1], 4, (255, 255, 255), -1)
        if line_grip_r:
            cv2.line(canvas, line_grip_r[0], line_grip_r[1], (0, 0, 255), 4)
            cv2.circle(canvas, line_grip_r[1], 4, (255, 255, 255), -1)

        # 6. Dibujar Articulaciones individuales
        colors_joints = [(50, 180, 255), (0, 230, 255), (0, 230, 255), (0, 180, 255)]
        for i, proj in enumerate(projs[1:]):
            if proj:
                cv2.circle(canvas, proj, 8, (0, 0, 0), -1)
                cv2.circle(canvas, proj, 6, colors_joints[i], -1)

        # 7. Información en pantalla (HUD de Cámara)
        cv2.putText(canvas, "REC [KINECT_POV]", (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        cv2.circle(canvas, (180, 28), 5, (0, 0, 255), -1) # Led de grabación
        
        cv2.putText(canvas, f"J1: {self.q[0]:.2f} rad", (20, 410), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        cv2.putText(canvas, f"J2: {self.q[1]:.2f} rad", (20, 430), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        cv2.putText(canvas, f"J3: {self.q[2]:.2f} rad", (20, 450), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        cv2.putText(canvas, f"J4: {self.q[3]:.2f} rad", (20, 470), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        
        # Coordenadas del efector final
        cv2.putText(canvas, f"EE Pos: X={p_ee[0]:.3f} Y={p_ee[1]:.3f} Z={p_ee[2]:.3f} m", (280, 460), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        return canvas

def main(args=None):
    rclpy.init(args=args)
    node = VisualizadorCamara()
    
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.03)
            canvas = node.draw_scene()
            
            cv2.imshow("Vista en Tiempo Real de Kinect (POV)", canvas)
            
            key = cv2.waitKey(30) & 0xFF
            if key == ord('q') or key == 27:
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
