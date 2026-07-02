#!/usr/bin/env python3
"""Percepcion 3D: Kinect color+depth -> Object3DArray en frame de camara.

Este nodo SOLO publica percepcion (brazo_interfaces/Object3DArray y
geometry_msgs/PointStamped en frame de camara). No decide alcanzabilidad
real ni mueve el brazo. La transformacion a base_link y la decision de
"reachable" respecto al workspace del brazo las hace camera_to_base_node
(ver brazo_ai). Aqui "reachable" solo indica que la lectura de profundidad
fue valida.

Publicar directamente en /target_position solo esta permitido para debug
manual con -p publish_direct_target:=true (ver FASE 2 del spec).
"""

import time
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, PointStamped
from cv_bridge import CvBridge

from brazo_interfaces.msg import Object3D, Object3DArray

from ultralytics import YOLO


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


class YoloDepthToPoint(Node):
    def __init__(self):
        super().__init__("yolo_depth_to_point")

        # Topics de la Kinect
        self.declare_parameter("color_topic", "/kinect2/color/image_raw")
        self.declare_parameter("depth_topic", "/kinect2/depth/image_raw")
        self.declare_parameter("depth_info_topic", "/kinect2/depth/camera_info")

        # Modelo / deteccion
        self.declare_parameter("target_class", "red")
        self.declare_parameter("model_path", "yolo11n.pt")
        self.declare_parameter("confidence", 0.45)
        self.declare_parameter("device", "cpu")  # 'cpu' recomendado en Jetson para evitar OOM de CUDA

        # Ventana para mediana de profundidad alrededor del centro del objeto
        self.declare_parameter("depth_window", 9)
        self.declare_parameter("min_depth", 0.20)
        self.declare_parameter("max_depth", 4.0)

        # Percepcion
        self.declare_parameter("camera_frame", "kinect2_depth_optical_frame")
        self.declare_parameter("publish_all_objects", True)
        self.declare_parameter("debug_annotations", True)

        # SOLO DEBUG. Nunca activar en modo autonomo / con LLM en el lazo.
        self.declare_parameter("publish_direct_target", False)

        # Offsets legacy camara->robot, usados UNICAMENTE en modo debug
        # (publish_direct_target:=true). La ruta principal usa TF2 en
        # camera_to_base_node, no estos offsets.
        self.declare_parameter("robot_x_offset", 0.0)
        self.declare_parameter("robot_y_offset", 0.0)
        self.declare_parameter("robot_z_offset", 1.20)

        # Limites de workspace legacy, solo para el clipping de debug
        self.x_min = -0.106
        self.x_max = 1.468
        self.y_min = -0.284
        self.y_max = 0.895
        self.z_min = 0.647
        self.z_max = 2.222
        self.r_min = 0.887
        self.r_max = 2.310

        self.bridge = CvBridge()

        self.depth_image = None
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.color_topic = self.get_parameter("color_topic").value
        self.depth_topic = self.get_parameter("depth_topic").value
        self.depth_info_topic = self.get_parameter("depth_info_topic").value

        self.target_class = self.get_parameter("target_class").value
        self.model_path = self.get_parameter("model_path").value
        self.confidence = float(self.get_parameter("confidence").value)
        self.camera_frame = self.get_parameter("camera_frame").value
        self.publish_direct_target = bool(self.get_parameter("publish_direct_target").value)
        self.publish_all_objects = bool(self.get_parameter("publish_all_objects").value)
        self.debug_annotations = bool(self.get_parameter("debug_annotations").value)

        self.model = None
        if self.target_class.lower() != "red":
            self.get_logger().info(f"Cargando modelo YOLO: {self.model_path}")
            self.model = YOLO(self.model_path)

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.create_subscription(Image, self.depth_topic, self.depth_callback, sensor_qos)
        self.create_subscription(CameraInfo, self.depth_info_topic, self.camera_info_callback, sensor_qos)
        self.create_subscription(Image, self.color_topic, self.color_callback, sensor_qos)

        self.objects_camera_pub = self.create_publisher(Object3DArray, "/perception/objects_camera", 10)
        self.selected_camera_pub = self.create_publisher(PointStamped, "/perception/selected_object_camera", 10)
        self.annotated_pub = self.create_publisher(Image, "/yolo/annotated_image", 10)

        # SOLO se usa si publish_direct_target == True.
        self.debug_target_pub = self.create_publisher(Point, "/target_position", 10)

        self.last_log_time = 0.0

        self.get_logger().info(
            "Nodo de percepcion listo. Publicando brazo_interfaces/Object3DArray en "
            "/perception/objects_camera (frame=%s)." % self.camera_frame
        )
        if self.publish_direct_target:
            self.get_logger().warn(
                "publish_direct_target=true: este nodo publicara /target_position "
                "directamente. SOLO usar para debug manual, nunca con el LLM en el lazo."
            )

    def camera_info_callback(self, msg):
        # Matriz K:
        # [fx  0 cx]
        # [ 0 fy cy]
        # [ 0  0  1]
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def detect_red_color(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)

        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

        mask = mask1 | mask2

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        boxes = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 400:  # area minima en pixeles
                x, y, w, h = cv2.boundingRect(cnt)
                boxes.append(([x, y, x + w, y + h], area))

        boxes.sort(key=lambda item: item[1], reverse=True)
        return boxes

    def depth_callback(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

            # El bridge publica 32FC1 en metros. Por seguridad, si llega
            # uint16 (mm), convertir a metros.
            if depth.dtype == np.uint16:
                depth = depth.astype(np.float32) * 0.001
            else:
                depth = depth.astype(np.float32)

            self.depth_image = depth

        except Exception as e:
            self.get_logger().error(f"Error leyendo depth: {e}")

    def get_median_depth(self, u, v):
        if self.depth_image is None:
            return None

        h, w = self.depth_image.shape[:2]
        if u < 0 or u >= w or v < 0 or v >= h:
            return None

        window = int(self.get_parameter("depth_window").value)
        half = max(1, window // 2)

        u1 = max(0, u - half)
        u2 = min(w, u + half + 1)
        v1 = max(0, v - half)
        v2 = min(h, v + half + 1)

        roi = self.depth_image[v1:v2, u1:u2]

        min_depth = float(self.get_parameter("min_depth").value)
        max_depth = float(self.get_parameter("max_depth").value)

        valid = roi[np.isfinite(roi)]
        valid = valid[(valid > min_depth) & (valid < max_depth)]

        if valid.size == 0:
            return None

        return float(np.median(valid))

    def legacy_camera_to_robot_frame(self, x_cam, y_cam, z_cam):
        """Transformacion aproximada por offsets. SOLO para debug manual.

        La ruta principal (autonoma/LLM) usa TF2 en camera_to_base_node.
        """
        robot_x_offset = float(self.get_parameter("robot_x_offset").value)
        robot_y_offset = float(self.get_parameter("robot_y_offset").value)
        robot_z_offset = float(self.get_parameter("robot_z_offset").value)

        x_robot = z_cam + robot_x_offset
        y_robot = -x_cam + robot_y_offset
        z_robot = -y_cam + robot_z_offset

        return x_robot, y_robot, z_robot

    def legacy_limit_workspace(self, x, y, z):
        x = clamp(x, self.x_min, self.x_max)
        y = clamp(y, self.y_min, self.y_max)
        z = clamp(z, self.z_min, self.z_max)

        r = (x * x + y * y) ** 0.5

        if r < 1e-6:
            x = self.r_min
            y = 0.0
        elif r < self.r_min:
            scale = self.r_min / r
            x *= scale
            y *= scale
        elif r > self.r_max:
            scale = self.r_max / r
            x *= scale
            y *= scale

        x = clamp(x, self.x_min, self.x_max)
        y = clamp(y, self.y_min, self.y_max)
        z = clamp(z, self.z_min, self.z_max)

        return x, y, z

    def make_object3d(self, header, object_id, class_name, confidence,
                       u_color, v_color, u_depth, v_depth, depth_m, bbox_area):
        """Construye un Object3D en frame de camara. reachable=True solo si
        la profundidad fue valida (no implica alcanzable por el brazo)."""
        obj = Object3D()
        obj.header = header
        obj.object_id = object_id
        obj.class_name = class_name
        obj.confidence = float(confidence)
        obj.u_color = int(u_color)
        obj.v_color = int(v_color)
        obj.u_depth = int(u_depth)
        obj.v_depth = int(v_depth)
        obj.bbox_area = float(bbox_area)

        if depth_m is None:
            obj.point.x = 0.0
            obj.point.y = 0.0
            obj.point.z = 0.0
            obj.depth_m = -1.0
            obj.reachable = False
            obj.reason = "invalid_depth"
            return obj

        x_cam = (u_depth - self.cx) * depth_m / self.fx
        y_cam = (v_depth - self.cy) * depth_m / self.fy
        z_cam = depth_m

        obj.point.x = float(x_cam)
        obj.point.y = float(y_cam)
        obj.point.z = float(z_cam)
        obj.depth_m = float(depth_m)
        obj.reachable = True
        obj.reason = "ok"
        return obj

    def publish_perception(self, header, objects):
        if not self.publish_all_objects:
            objects = [o for o in objects if o.reachable]

        arr = Object3DArray()
        arr.header = header
        arr.objects = objects
        self.objects_camera_pub.publish(arr)

        valid_objects = [o for o in objects if o.reachable]
        if not valid_objects:
            return

        selected = max(valid_objects, key=lambda o: o.bbox_area)

        selected_point = PointStamped()
        selected_point.header = header
        selected_point.point = selected.point
        self.selected_camera_pub.publish(selected_point)

        if self.publish_direct_target:
            x_robot_raw, y_robot_raw, z_robot_raw = self.legacy_camera_to_robot_frame(
                selected.point.x, selected.point.y, selected.point.z
            )
            x_robot, y_robot, z_robot = self.legacy_limit_workspace(
                x_robot_raw, y_robot_raw, z_robot_raw
            )
            debug_point = Point()
            debug_point.x = float(x_robot)
            debug_point.y = float(y_robot)
            debug_point.z = float(z_robot)
            self.debug_target_pub.publish(debug_point)

        now = time.time()
        if now - self.last_log_time > 0.5:
            self.get_logger().info(
                f"Objetos detectados: {len(objects)} (validos: {len(valid_objects)}) | "
                f"seleccionado={selected.object_id} cam=({selected.point.x:.3f}, "
                f"{selected.point.y:.3f}, {selected.point.z:.3f})"
            )
            self.last_log_time = now

    def color_callback(self, msg):
        if self.depth_image is None or self.fx is None:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Error leyendo color: {e}")
            return

        header = msg.header
        header.frame_id = self.camera_frame

        color_h, color_w = frame.shape[:2]
        depth_h, depth_w = self.depth_image.shape[:2]

        annotated_frame = frame.copy() if self.debug_annotations else None
        objects = []

        if self.target_class.lower() == "red":
            boxes = self.detect_red_color(frame)
            for idx, (box, area) in enumerate(boxes):
                bx1, by1, bx2, by2 = box
                u_color = int((bx1 + bx2) / 2.0)
                v_color = int((by1 + by2) / 2.0)
                u_depth = int(u_color * depth_w / color_w)
                v_depth = int(v_color * depth_h / color_h)

                depth_m = self.get_median_depth(u_depth, v_depth)
                obj = self.make_object3d(
                    header, f"red_{idx}", "red", 1.0,
                    u_color, v_color, u_depth, v_depth, depth_m, area
                )
                objects.append(obj)

                if annotated_frame is not None:
                    color = (0, 0, 255) if obj.reachable else (128, 128, 128)
                    label = f"{obj.object_id} ({obj.point.x:.2f},{obj.point.y:.2f},{obj.point.z:.2f})"
                    cv2.rectangle(annotated_frame, (bx1, by1), (bx2, by2), color, 2)
                    cv2.putText(annotated_frame, label, (bx1, max(0, by1 - 10)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        else:
            device = self.get_parameter("device").value
            result = self.model.predict(
                source=frame,
                imgsz=640,
                conf=self.confidence,
                device=device,
                verbose=False
            )[0]

            names = result.names
            if result.boxes is not None:
                for idx, box in enumerate(result.boxes):
                    cls_id = int(box.cls[0])
                    conf = float(box.conf[0])
                    class_name = names[cls_id]

                    if self.target_class != "" and class_name != self.target_class:
                        continue

                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    bx1, by1, bx2, by2 = int(x1), int(y1), int(x2), int(y2)
                    u_color = int((bx1 + bx2) / 2.0)
                    v_color = int((by1 + by2) / 2.0)
                    u_depth = int(u_color * depth_w / color_w)
                    v_depth = int(v_color * depth_h / color_h)
                    area = float((bx2 - bx1) * (by2 - by1))

                    depth_m = self.get_median_depth(u_depth, v_depth)
                    obj = self.make_object3d(
                        header, f"{class_name}_{idx}", class_name, conf,
                        u_color, v_color, u_depth, v_depth, depth_m, area
                    )
                    objects.append(obj)

            if annotated_frame is not None:
                try:
                    annotated_frame = result.plot()
                except Exception as e:
                    self.get_logger().error(f"Error generando imagen anotada: {e}")

        if objects:
            self.publish_perception(header, objects)

        if annotated_frame is not None:
            try:
                annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
                self.annotated_pub.publish(annotated_msg)
            except Exception as e:
                self.get_logger().error(f"Error publicando imagen anotada: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = YoloDepthToPoint()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
