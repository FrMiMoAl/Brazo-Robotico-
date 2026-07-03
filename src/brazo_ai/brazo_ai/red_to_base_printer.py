#!/usr/bin/env python3
"""
red_to_base_printer.py

Suscribe a /perception/selected_object_camera (PointStamped en frame de cámara)
y convierte la posición 3D al frame base_link usando la geometría física real
del setup: Kinect Xbox 1 en el suelo, 15 cm detrás del robot, centrada,
inclinada hacia arriba.

Convención de ejes kinect2_depth_optical_frame:
  Z → profundidad (hacia adelante de la lente)
  X → derecha en la imagen
  Y → abajo en la imagen

Convención de ejes base_link (ROS estándar):
  X → hacia adelante del robot
  Y → hacia la izquierda del robot
  Z → hacia arriba

Ajuste del ángulo de tilt: modificar TILT_DEG hasta que Z_base sea positivo
y coherente con la altura real del objeto sobre el suelo.

Uso:
    ros2 run brazo_ai red_to_base_printer
    ros2 run brazo_ai red_to_base_printer --ros-args -p tilt_deg:=30.0
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import PointStamped

# ─────────────────────────────────────────────────────────────────
#  Parámetros físicos del setup (ajustar según medición real)
# ─────────────────────────────────────────────────────────────────

# Posición de la cámara en base_link [metros]
# La cámara está 15 cm detrás del robot, misma altura, centrada
CAM_X = -0.15   # detrás del robot → -X
CAM_Y =  0.0    # centrada → Y=0
CAM_Z =  0.0    # misma altura que base_link → Z=0

# Ángulo de tilt vertical de la Kinect (positivo = inclinada hacia arriba)
# Empezar con 25° y ajustar: si Z_base sale negativo → aumentar el ángulo
TILT_DEG = 25.0


def _build_rotation(tilt_deg: float) -> np.ndarray:
    """
    Construye la matriz de rotación R tal que:
        p_base = R @ p_cam + t

    La cámara mira hacia +X_base (hacia el workspace del robot)
    inclinada TILT_DEG grados hacia arriba.

    Sin tilt (0°):
      Z_cam → +X_base  (profundidad = avance del brazo)
      X_cam → -Y_base  (derecha imagen = -Y base, ROS Y apunta a la izquierda)
      Y_cam → -Z_base  (abajo imagen = abajo físico)

    Con tilt alpha hacia arriba (rotar alrededor del eje lateral = X_cam):
      Z_cam → (cos(a), 0,  sin(a))  en base
      X_cam → (0,     -1,  0     )  en base  [sin cambio]
      Y_cam → (sin(a), 0, -cos(a))  en base
    """
    a = math.radians(tilt_deg)
    ca, sa = math.cos(a), math.sin(a)

    # Columnas = donde va cada eje de cámara en base_link
    # R[:, 0] = X_cam en base
    # R[:, 1] = Y_cam en base
    # R[:, 2] = Z_cam en base
    R = np.array([
        # X_cam_base   Y_cam_base   Z_cam_base
        [0,             sa,          ca],   # fila base X
        [-1,            0,           0 ],   # fila base Y
        [0,            -ca,          sa],   # fila base Z
    ])
    return R


def cam_to_base(R: np.ndarray, t: np.ndarray,
                x_cam: float, y_cam: float, z_cam: float):
    """Transforma un punto de frame cámara a base_link."""
    p_cam = np.array([x_cam, y_cam, z_cam])
    p_base = R @ p_cam + t
    return float(p_base[0]), float(p_base[1]), float(p_base[2])


# ─────────────────────────────────────────────────────────────────
#  Nodo ROS 2
# ─────────────────────────────────────────────────────────────────
class RedToBasePrinter(Node):

    def __init__(self):
        super().__init__('red_to_base_printer')

        # Parámetros ajustables en tiempo de ejecución
        self.declare_parameter('tilt_deg', TILT_DEG)
        self.declare_parameter('cam_x', CAM_X)
        self.declare_parameter('cam_y', CAM_Y)
        self.declare_parameter('cam_z', CAM_Z)

        tilt = self.get_parameter('tilt_deg').value
        cx   = self.get_parameter('cam_x').value
        cy   = self.get_parameter('cam_y').value
        cz   = self.get_parameter('cam_z').value

        self._R = _build_rotation(tilt)
        self._t = np.array([cx, cy, cz])

        self.get_logger().info(
            f'\n'
            f'╔══════════════════════════════════════════════════════════╗\n'
            f'║         red_to_base_printer  (transformación física)    ║\n'
            f'╠══════════════════════════════════════════════════════════╣\n'
            f'║  Setup: cámara a ({cx:.2f}, {cy:.2f}, {cz:.2f}) m en base_link       ║\n'
            f'║  Tilt: {tilt:.1f}° hacia arriba                               ║\n'
            f'║  Suscrito: /perception/selected_object_camera           ║\n'
            f'║  AJUSTE: si Z_base<0 → aumentar tilt_deg               ║\n'
            f'╚══════════════════════════════════════════════════════════╝'
        )

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.create_subscription(
            PointStamped,
            '/perception/selected_object_camera',
            self._callback,
            qos,
        )

        self._prev = None
        self._count = 0

    def _callback(self, msg: PointStamped):
        xc = msg.point.x
        yc = msg.point.y
        zc = msg.point.z

        xb, yb, zb = cam_to_base(self._R, self._t, xc, yc, zc)

        # Suprimir si la posición no cambió más de 1 cm
        curr = (round(xb, 2), round(yb, 2), round(zb, 2))
        if curr == self._prev:
            return
        self._prev = curr
        self._count += 1

        # Advertencia si Z base es negativo (tilt insuficiente)
        z_warn = '  ⚠ Z<0 → aumentar tilt_deg' if zb < 0 else ''

        print(
            f'\n[#{self._count:04d}]'
            f'  CAM  x={xc:+.3f}  y={yc:+.3f}  z={zc:+.3f} m\n'
            f'        BASE x={xb:+.3f}  y={yb:+.3f}  z={zb:+.3f} m'
            f'{z_warn}'
        )


# ─────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = RedToBasePrinter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
