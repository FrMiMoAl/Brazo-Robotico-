#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image, CameraInfo
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
import numpy as np

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

class BrazoController(Node):
    def __init__(self):
        super().__init__('control_brazo')
        
        # Publicador de estados articulares
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Suscriptor a la posición objetivo
        self.target_sub = self.create_subscription(
            Point,
            '/target_position',
            self.target_callback,
            10
        )
        
        # Suscriptor al control del eyector (gripper)
        self.gripper_sub = self.create_subscription(
            Bool,
            '/gripper_command',
            self.gripper_callback,
            10
        )
        
        # Suscriptor a comandos directos de articulaciones
        self.joint_cmd_sub = self.create_subscription(
            JointState,
            '/joint_commands',
            self.joint_command_callback,
            10
        )
        
        # Publicadores para simulación de cámara Kinect en RViz (Camera Overlay)
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
        
        # Pre-generar los datos de la imagen (640x480 gris claro)
        self.dummy_image_data = bytes([245] * (640 * 480 * 3))
        self.camera_publish_counter = 0
        
        # Ángulos articulares actuales [q1, q2, q3, q4] (Home: J1=0.0, J2=0.0, J3=0.0, J4=0.0 deg)
        self.q_current = np.array([0.0, 0.0, 0.0, 0.0])
        
        # Cola de trayectoria (lista de arrays de 4 elementos)
        self.trajectory_queue = []
        
        # Timer para publicar estados y ejecutar trayectorias (50 Hz -> 0.02s)
        self.dt = 0.02
        self.timer = self.create_timer(self.dt, self.timer_callback)
        
        # Parámetros físicos del robot (de acuerdo a los eslabones y URDF)
        self.joint_limits = [
            (-np.pi, np.pi),                          # J1
            (np.radians(-140), np.radians(0)),        # J2 (-140 a 0)
            (0.0, np.radians(150)),                   # J3 (0 a 150)
            (0.0, np.radians(180))                    # J4 (0 a 180)
        ]
        
        # Nombres de las articulaciones del URDF
        self.joint_names = [
            '1', '2', '3', '4',
            'left_joint', 'right_joint',
            'left_gear', 'right_gear_joint'
        ]
        
        # Parámetros del nuevo gripper
        self.q_gripper = 0.0
        self.q_gripper_open = -0.2
        self.q_gripper_closed = 0.6

        # Definición de las articulaciones según el nuevo URDF sin gripper
        self.joints_data = [
            {
                'xyz': [0.045435, -0.019492, 0.042792],
                'rpy': [0.000000, 0.000000, -0.495569],
                'axis': [0.000000, 0.000000, 1.000000]
            },
            {
                'xyz': [0.000000, 0.000000, 0.093058],
                'rpy': [1.542194, -1.047020, 2.119165],
                'axis': [0.500000, -0.866025, 0.000000]
            },
            {
                'xyz': [0.118563, 0.068452, 0.376872],
                'rpy': [0.000000, 1.150442, 0.523599],
                'axis': [0.000000, -1.000000, 0.000000]
            },
            {
                'xyz': [0.23364969, 0.000000, -0.04785507],
                'rpy': [0.000000, 2.78230065, 0.000000],
                'axis': [0.000000, 1.000000, 0.000000]
            }
        ]
        
        self.get_logger().info('Nodo control_brazo iniciado. Listo para recibir objetivos en /target_position y /gripper_command.')

    def gripper_callback(self, msg):
        if msg.data:
            self.q_gripper = self.q_gripper_open
            self.get_logger().info('Gripper: ABIERTO')
        else:
            self.q_gripper = self.q_gripper_closed
            self.get_logger().info('Gripper: CERRADO')

    def forward_kinematics_ee(self, q):
        """
        Calcula la cinemática directa desde la base hasta la punta del link 4 (sin gripper).
        q: [q1, q2, q3, q4]
        Retorna la posición [x, y, z] en metros en el frame base_link.
        """
        T = np.eye(4)
        for i in range(4):
            jd = self.joints_data[i]
            T_joint = joint_transform(jd['xyz'], jd['rpy'], jd['axis'], q[i])
            T = T @ T_joint
        
        # Añadir offset de la punta (8 cm en X del link 4)
        T_tip = np.eye(4)
        T_tip[0, 3] = 0.08
        T = T @ T_tip
        return T[:3, 3]

    def inverse_kinematics(self, target_pos, max_iters=150, tol=1e-4):
        """
        Resuelve la cinemática inversa numéricamente mediante Levenberg-Marquardt amortiguado.
        Soporta junta J1 continua sin límites físicos usando wrap-around.
        """
        q = self.q_current.copy()
        damping = 0.005
        
        for i in range(max_iters):
            # Envolver J1 (index 0) en el rango [-pi, pi]
            q[0] = np.arctan2(np.sin(q[0]), np.cos(q[0]))
            
            pos = self.forward_kinematics_ee(q)
            error = target_pos - pos
            
            if np.linalg.norm(error) < tol:
                q[0] = np.arctan2(np.sin(q[0]), np.cos(q[0]))
                return q, True
            
            # Calcular Jacobiano geométrico numéricamente
            J = np.zeros((3, 4))
            epsilon = 1e-6
            for j in range(4):
                q_perturbed = q.copy()
                q_perturbed[j] += epsilon
                pos_perturbed = self.forward_kinematics_ee(q_perturbed)
                J[:, j] = (pos_perturbed - pos) / epsilon
            
            # Jacobiano pseudo-inverso con amortiguamiento
            JJt = J @ J.T
            J_pseudo = J.T @ np.linalg.inv(JJt + damping * np.eye(3))
            dq = J_pseudo @ error
            
            # Actualización
            q += dq
            
            # Limitar a los rangos físicos (solo joints 2, 3, 4 ya que J1 es continua)
            for j in range(1, 4):
                low, high = self.joint_limits[j]
                q[j] = np.clip(q[j], low, high)
                
        # Verificación y empaquetado final
        q[0] = np.arctan2(np.sin(q[0]), np.cos(q[0]))
        pos_final = self.forward_kinematics_ee(q)
        if np.linalg.norm(target_pos - pos_final) < 0.01:
            return q, True
        return q, False

    def target_callback(self, msg):
        target_pos = np.array([msg.x, msg.y, msg.z])
        self.get_logger().info(f'Recibido objetivo: X={msg.x:.3f}, Y={msg.y:.3f}, Z={msg.z:.3f}')
        
        q_sol, success = self.inverse_kinematics(target_pos)
        
        if success:
            self.get_logger().info(f'¡Solución de IK encontrada! Joints: {np.round(q_sol, 4)}')
            
            # Calcular la diferencia de juntas considerando el wrap-around para J1 (index 0)
            dq = q_sol - self.q_current
            dq[0] = np.arctan2(np.sin(dq[0]), np.cos(dq[0]))
            
            # Calculamos la diferencia angular máxima para decidir la duración
            max_diff = np.max(np.abs(dq))
            max_speed = 0.5  # rad/s max velocidad
            
            duration = max(max_diff / max_speed, 1.0)  # Mínimo 1 segundo de movimiento
            steps = int(duration / self.dt)
            
            # Generar los pasos de la trayectoria
            self.trajectory_queue = []
            for step in range(1, steps + 1):
                alpha = step / steps
                q_step = self.q_current + alpha * dq
                q_step[0] = np.arctan2(np.sin(q_step[0]), np.cos(q_step[0]))
                self.trajectory_queue.append(q_step)
                
            self.get_logger().info(f'Trayectoria planeada con {steps} pasos en {duration:.2f} segundos.')
        else:
            self.get_logger().error('No se pudo encontrar una solución de IK válida para el punto objetivo.')

    def joint_command_callback(self, msg):
        if len(msg.position) >= 4:
            q_sol = np.array(msg.position[:4])
            
            # Envolver J1 (index 0) en [-pi, pi]
            q_sol[0] = np.arctan2(np.sin(q_sol[0]), np.cos(q_sol[0]))
            
            # Limitar a los rangos físicos (solo joints 2, 3, 4 ya que J1 es continua)
            for j in range(1, 4):
                low, high = self.joint_limits[j]
                q_sol[j] = np.clip(q_sol[j], low, high)
                
            # Calcular la diferencia de juntas considerando el wrap-around para J1 (index 0)
            dq = q_sol - self.q_current
            dq[0] = np.arctan2(np.sin(dq[0]), np.cos(dq[0]))
            
            # Calculamos la diferencia angular máxima para decidir la duración
            max_diff = np.max(np.abs(dq))
            max_speed = 0.5  # rad/s max velocidad
            
            duration = max(max_diff / max_speed, 1.0)  # Mínimo 1 segundo de movimiento
            steps = int(duration / self.dt)
            
            # Generar los pasos de la trayectoria
            self.trajectory_queue = []
            for step in range(1, steps + 1):
                alpha = step / steps
                q_step = self.q_current + alpha * dq
                q_step[0] = np.arctan2(np.sin(q_step[0]), np.cos(q_step[0]))
                self.trajectory_queue.append(q_step)
                
            self.get_logger().info(f'Recibido comando articular directo: {np.round(q_sol, 4)}. Trayectoria planeada.')

    def timer_callback(self):
        # Si hay trayectoria en la cola, avanzar al siguiente paso
        if len(self.trajectory_queue) > 0:
            self.q_current = self.trajectory_queue.pop(0)
            
        # Asegurar que la articulación J1 se mantenga en [-pi, pi]
        self.q_current[0] = np.arctan2(np.sin(self.q_current[0]), np.cos(self.q_current[0]))
            
        # Publicar el JointState
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        
        # Posición de los 4 joints principales + los joints del gripper
        msg.position = [
            self.q_current[0],
            self.q_current[1],
            self.q_current[2],
            self.q_current[3],
            self.q_gripper,            # left_joint
            self.q_gripper,            # right_joint
            0.41 * self.q_gripper,     # left_gear
            0.41 * self.q_gripper      # right_gear_joint
        ]
        
        self.joint_pub.publish(msg)

        # Publicar cámara a 10 Hz (cada 5 ciclos de self.dt = 0.02s)
        self.camera_publish_counter += 1
        if self.camera_publish_counter >= 5:
            self.camera_publish_counter = 0
            self.publish_camera_overlay(msg.header.stamp)

    def publish_camera_overlay(self, stamp):
        img_msg = Image()
        img_msg.header.stamp = stamp
        img_msg.header.frame_id = "kinect_optical_frame"
        img_msg.height = 480
        img_msg.width = 640
        img_msg.encoding = "rgb8"
        img_msg.is_bigendian = 0
        img_msg.step = 640 * 3
        img_msg.data = self.dummy_image_data
        
        info_msg = CameraInfo()
        info_msg.header = img_msg.header
        info_msg.width = 640
        info_msg.height = 480
        
        f = 500.0
        info_msg.k = [f, 0.0, 320.0, 0.0, f, 240.0, 0.0, 0.0, 1.0]
        info_msg.p = [f, 0.0, 320.0, 0.0, 0.0, f, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        
        self.image_pub.publish(img_msg)
        self.info_pub.publish(info_msg)

def main(args=None):
    rclpy.init(args=args)
    node = BrazoController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
