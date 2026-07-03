#!/usr/bin/env python3
"""
tag_calibrator.py

* Suscribe a la imagen color de la Kinect (topic /kinect2/color/image_raw).
* Detecta **ArUco** o **AprilTag** (según parámetro `marker_family`).
* Estima la pose del marcador usando `solvePnP`.
* Guarda cada par (pose_camera ↔ pose_base) en un archivo de texto o lo publica
  como `geometry_msgs/PoseStamped` en el frame `kinect2_depth_optical_frame`.
* Cuando se recogen `n_samples` pares, ejecuta el algoritmo Kabsch y muestra
  la transformación estática `base_link → kinect2_depth_optical_frame`.

Ejemplo de ejecución:

    ros2 run brazo_ai tag_calibrator \
        --ros-args -p marker_family:=aruco \
                  -p marker_id:=0 \
                  -p marker_size:=0.05 \
                  -p n_samples:=6 \
                  -p output_file:=aruco_calib.txt
"""

import os
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PointStamped

import cv2

# ---------------------------------------------------------------
# AprilTag optional import
# ---------------------------------------------------------------
try:
    import apriltag
    HAVE_APRILTAG = True
except Exception:
    HAVE_APRILTAG = False

# ---------------------------------------------------------------
# Kabsch calibration (reuse from previous script)
# ---------------------------------------------------------------
def kabsch_calibration(points_camera, points_base):
    """Devuelve rotación R (3×3) y traslación t (3,) que alinean cámara → base."""
    c_cam = np.mean(points_camera, axis=0)
    c_base = np.mean(points_base, axis=0)
    cam_centered = points_camera - c_cam
    base_centered = points_base - c_base
    H = cam_centered.T @ base_centered
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[2, :] *= -1
        R = Vt.T @ U.T
    t = c_base - R @ c_cam
    return R, t


def rotation_matrix_to_euler(R):
    """Convierte a roll‑pitch‑yaw (ROS X‑Y‑Z)."""
    pitch = np.arctan2(-R[2, 0], np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2))
    if np.abs(pitch - np.pi / 2) > 1e-5 and np.abs(pitch + np.pi / 2) > 1e-5:
        roll = np.arctan2(R[2, 1], R[2, 2])
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll = 0.0
        yaw = np.arctan2(-R[0, 1], R[1, 1])
    return roll, pitch, yaw


class TagCalibrator(Node):
    def __init__(self):
        super().__init__('tag_calibrator')

        # ---- Parámetros ----
        self.declare_parameter('marker_family', 'aruco')   # aruco | apriltag
        self.declare_parameter('marker_id', 0)            # solo para aruco
        self.declare_parameter('marker_size', 0.05)       # metros
        self.declare_parameter('n_samples', 6)            # cuántas pares
        self.declare_parameter('output_file', '')         # opcional CSV

        self.marker_family = self.get_parameter('marker_family').value.lower()
        self.marker_id = int(self.get_parameter('marker_id').value)
        self.marker_size = float(self.get_parameter('marker_size').value)
        self.n_samples = int(self.get_parameter('n_samples').value)
        self.output_file = self.get_parameter('output_file').value

        # ---- Estado interno ----
        self.samples_cam = []   # (X,Y,Z) en cámara
        self.samples_base = []  # (X,Y,Z) en base

        # ---- ROS interfaces ----
        self.bridge = CvBridge()
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        if self.marker_family != 'red':
            self.create_subscription(Image, '/kinect2/color/image_raw', self.image_cb, qos)
            self.create_subscription(CameraInfo, '/kinect2/color/camera_info', self.camera_info_cb, qos)
        else:
            self.create_subscription(PointStamped, '/perception/selected_object_camera', self.selected_object_cb, qos)
        self.pose_pub = self.create_publisher(PoseStamped, '/detected_tag/pose_camera', 10)

        # ---- Detectores ----
        if self.marker_family == 'aruco':
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            if hasattr(cv2.aruco, 'DetectorParameters_create'):
                self.aruco_params = cv2.aruco.DetectorParameters_create()
            else:
                self.aruco_params = cv2.aruco.DetectorParameters()
        elif self.marker_family == 'apriltag':
            if not HAVE_APRILTAG:
                self.get_logger().error('apriltag library not installed. Install with: pip3 install apriltag')
                sys.exit(1)
            self.apriltag_detector = apriltag.Detector()
        elif self.marker_family == 'red':
            pass
        else:
            self.get_logger().error(f'Familia de marcador desconocida: {self.marker_family}')
            sys.exit(1)

        self.camera_matrix = None
        self.dist_coeffs = None
        self.get_logger().info('Nodo TagCalibrator inicializado. Esperando detecciones.')

    # ------------------------------------------------------------------
    def selected_object_cb(self, msg):
        self.last_cam_pose = [msg.point.x, msg.point.y, msg.point.z]

    # ------------------------------------------------------------------
    def camera_info_cb(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info('Matriz de cámara recibida.')

    # ------------------------------------------------------------------
    def image_cb(self, msg):
        if self.camera_matrix is None:
            return  # No hay info de cámara todavía
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Conversión de imagen falló: {e}')
            return
        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)

        # -------- Detección ----------
        if self.marker_family == 'aruco':
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
            if ids is None or self.marker_id not in ids:
                return
            idx = list(ids.flatten()).index(self.marker_id)
            marker_corners = corners[idx]
            retval, rvec, tvec = cv2.aruco.estimatePoseSingleMarkers(
                marker_corners, self.marker_size, self.camera_matrix, self.dist_coeffs)
            if retval is None:
                return
            cam_pose = tvec[0, 0]
            rot_vec = rvec[0, 0]
        else:  # apriltag
            detections = self.apriltag_detector.detect(gray)
            if not detections:
                return
            detection = detections[0]
            img_pts = np.array(detection.corners, dtype=np.float32)
            half = self.marker_size / 2.0
            obj_pts = np.array([
                [-half, -half, 0],
                [ half, -half, 0],
                [ half,  half, 0],
                [-half,  half, 0],
            ], dtype=np.float32)
            success, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, self.camera_matrix, self.dist_coeffs,
                                            flags=cv2.SOLVEPNP_ITERATIVE)
            if not success:
                return
            cam_pose = tvec.ravel()
            rot_vec = rvec.ravel()

        # -------- Publicar pose ----------
        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        pose_msg.header.frame_id = 'kinect2_depth_optical_frame'
        pose_msg.pose.position.x = float(cam_pose[0])
        pose_msg.pose.position.y = float(cam_pose[1])
        pose_msg.pose.position.z = float(cam_pose[2])
        rot_mat, _ = cv2.Rodrigues(rot_vec)
        quat = self.rotmat_to_quaternion(rot_mat)
        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]
        self.pose_pub.publish(pose_msg)
        self.last_cam_pose = cam_pose.tolist()

    # ------------------------------------------------------------------
    @staticmethod
    def rotmat_to_quaternion(R):
        q = np.empty(4)
        trace = np.trace(R)
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            q[3] = 0.25 / s
            q[0] = (R[2, 1] - R[1, 2]) * s
            q[1] = (R[0, 2] - R[2, 0]) * s
            q[2] = (R[1, 0] - R[0, 1]) * s
        else:
            if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
                s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
                q[3] = (R[2, 1] - R[1, 2]) / s
                q[0] = 0.25 * s
                q[1] = (R[0, 1] + R[1, 0]) / s
                q[2] = (R[0, 2] + R[2, 0]) / s
            elif R[1, 1] > R[2, 2]:
                s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
                q[3] = (R[0, 2] - R[2, 0]) / s
                q[0] = (R[0, 1] + R[1, 0]) / s
                q[1] = 0.25 * s
                q[2] = (R[1, 2] + R[2, 1]) / s
            else:
                s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
                q[3] = (R[1, 0] - R[0, 1]) / s
                q[0] = (R[0, 2] + R[2, 0]) / s
                q[1] = (R[1, 2] + R[2, 1]) / s
                q[2] = 0.25 * s
        return q

    # ------------------------------------------------------------------
    def collect_samples(self):
        while rclpy.ok() and len(self.samples_cam) < self.n_samples:
            input('\n> Colocá el marcador y pulsá <Enter> para registrar →')
            
            # Clear previous pose to guarantee a fresh detection
            if hasattr(self, 'last_cam_pose'):
                delattr(self, 'last_cam_pose')
                
            # Spin for up to 3 seconds until a fresh pose is received
            start_t = self.get_clock().now()
            while rclpy.ok() and not hasattr(self, 'last_cam_pose'):
                rclpy.spin_once(self, timeout_sec=0.1)
                elapsed = (self.get_clock().now() - start_t).nanoseconds / 1e9
                if elapsed > 3.0:
                    break
                    
            if not hasattr(self, 'last_cam_pose'):
                self.get_logger().error('Aún no se detectó el marcador en la nueva posición. Inténtalo de nuevo.')
                continue
            cam_xyz = self.last_cam_pose
            self.samples_cam.append(cam_xyz)
            self.get_logger().info(f'Pose cámara guardada: {cam_xyz}')
            while True:
                try:
                    txt = input('   Coordenadas en BASE (X Y Z en metros): ')
                    xb, yb, zb = map(float, txt.strip().split())
                    self.samples_base.append([xb, yb, zb])
                    break
                except ValueError:
                    print('   [ERROR] Formato inválido. Ingresa tres números separados por espacio.')

        # --- Kabsch ---
        A = np.array(self.samples_cam)
        B = np.array(self.samples_base)
        R, t = kabsch_calibration(A, B)
        roll, pitch, yaw = rotation_matrix_to_euler(R)
        aligned = (A @ R.T) + t
        rmsd = np.sqrt(np.mean(np.linalg.norm(aligned - B, axis=1) ** 2))
        self.get_logger().info('\n===== RESULTADOS DE CALIBRACIÓN =====')
        self.get_logger().info(f'Error RMSD: {rmsd*100:.2f} cm')
        if rmsd > 0.03:
            self.get_logger().warning('Error > 3 cm → revisá las mediciones.')
        else:
            self.get_logger().info('Calibración de alta calidad.')
        self.get_logger().info(f'Translación: X={t[0]:.4f} m  Y={t[1]:.4f} m  Z={t[2]:.4f} m')
        self.get_logger().info(f'Rotación (rad): Roll={roll:.4f} Pitch={pitch:.4f} Yaw={yaw:.4f}')
        self.get_logger().info(f'Rotación (deg): Roll={np.degrees(roll):.1f}° Pitch={np.degrees(pitch):.1f}° Yaw={np.degrees(yaw):.1f}°')
        cmd = (f'ros2 run tf2_ros static_transform_publisher '
               f'{t[0]:.4f} {t[1]:.4f} {t[2]:.4f} '
               f'{roll:.4f} {pitch:.4f} {yaw:.4f} '
               f'base_link kinect2_depth_optical_frame')
        self.get_logger().info('COPIÁ y EJECUTÁ este comando para publicar la transformación:')
        self.get_logger().info(cmd)
        if self.output_file:
            with open(self.output_file, 'w') as f:
                f.write('# camera_X camera_Y camera_Z  base_X base_Y base_Z\n')
                for cam, base in zip(self.samples_cam, self.samples_base):
                    f.write(f'{cam[0]} {cam[1]} {cam[2]} {base[0]} {base[1]} {base[2]}\n')
                f.write('# Transformación resultante\n')
                f.write(f'translation: {t[0]} {t[1]} {t[2]}\n')
                f.write(f'rotation_rpy: {roll} {pitch} {yaw}\n')
            self.get_logger().info(f'Resultados guardados en: {self.output_file}')
        rclpy.shutdown()


def main():
    rclpy.init()
    node = TagCalibrator()
    # Esperar a que llegue al menos una pose válida antes de iniciar la captura
    while rclpy.ok() and not hasattr(node, 'last_cam_pose'):
        rclpy.spin_once(node, timeout_sec=0.1)
    node.collect_samples()

if __name__ == '__main__':
    main()
