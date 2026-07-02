#!/usr/bin/env python3
"""Transforma objetos detectados de frame de camara a base_link via TF2.

Entrada:  /perception/objects_camera   (brazo_interfaces/Object3DArray)
Salidas:  /perception/objects_base     (brazo_interfaces/Object3DArray)
          /perception/selected_object_base (geometry_msgs/PointStamped)

Esta es la UNICA ruta principal de transformacion camara->robot. No usa
offsets fijos: requiere que exista la transformacion TF2
base_link <- kinect2_depth_optical_frame (ver FASE 5 del spec, publicada
por ejemplo con tf2_ros static_transform_publisher mientras no haya una
calibracion definitiva).

"reachable" aqui ya tiene en cuenta el workspace real del brazo. Si la
transformacion TF no esta disponible, este nodo NO publica (no se debe
clipear ni inventar una posicion).
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

import tf2_ros
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped

from brazo_interfaces.msg import Object3D, Object3DArray


class CameraToBaseNode(Node):
    def __init__(self):
        super().__init__("camera_to_base_node")

        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("camera_frame", "kinect2_depth_optical_frame")
        self.declare_parameter("workspace_x_min", 0.05)
        self.declare_parameter("workspace_x_max", 0.35)
        self.declare_parameter("workspace_y_min", -0.25)
        self.declare_parameter("workspace_y_max", 0.25)
        self.declare_parameter("workspace_z_min", 0.02)
        self.declare_parameter("workspace_z_max", 0.35)
        self.declare_parameter("object_stale_timeout", 0.75)

        self.base_frame = self.get_parameter("base_frame").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.object_stale_timeout = float(self.get_parameter("object_stale_timeout").value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(
            Object3DArray, "/perception/objects_camera", self.objects_callback, 10
        )
        self.objects_base_pub = self.create_publisher(Object3DArray, "/perception/objects_base", 10)
        self.selected_base_pub = self.create_publisher(PointStamped, "/perception/selected_object_base", 10)

        self._last_tf_warn = 0.0

        self.get_logger().info(
            f"camera_to_base_node listo. Esperando TF {self.base_frame} <- {self.camera_frame}."
        )

    def workspace_check(self, x, y, z):
        x_min = float(self.get_parameter("workspace_x_min").value)
        x_max = float(self.get_parameter("workspace_x_max").value)
        y_min = float(self.get_parameter("workspace_y_min").value)
        y_max = float(self.get_parameter("workspace_y_max").value)
        z_min = float(self.get_parameter("workspace_z_min").value)
        z_max = float(self.get_parameter("workspace_z_max").value)

        if not (x_min <= x <= x_max):
            return False, "outside_workspace"
        if not (y_min <= y <= y_max):
            return False, "outside_workspace"
        if not (z_min <= z <= z_max):
            return False, "outside_workspace"
        return True, "ok"

    def warn_no_tf(self, detail):
        now = time.time()
        if now - self._last_tf_warn > 2.0:
            self.get_logger().warn(
                f"No transform from {self.camera_frame} to {self.base_frame}. "
                f"Calibrate TF first. ({detail})"
            )
            self._last_tf_warn = now

    def objects_callback(self, msg: Object3DArray):
        if not msg.objects:
            return

        stamp = msg.header.stamp
        age = self.get_clock().now() - rclpy.time.Time.from_msg(stamp)
        if age.nanoseconds / 1e9 > self.object_stale_timeout:
            return  # percepcion vieja, no propagar (evitar perseguir ruido/estado obsoleto)

        source_frame = msg.header.frame_id or self.camera_frame

        if not self.tf_buffer.can_transform(
            self.base_frame, source_frame, rclpy.time.Time(), timeout=Duration(seconds=0.05)
        ):
            self.warn_no_tf("can_transform=False")
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, source_frame, rclpy.time.Time()
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.warn_no_tf(str(e))
            return

        out_objects = []
        for obj in msg.objects:
            new_obj = Object3D()
            new_obj.header.stamp = msg.header.stamp
            new_obj.header.frame_id = self.base_frame
            new_obj.object_id = obj.object_id
            new_obj.class_name = obj.class_name
            new_obj.confidence = obj.confidence
            new_obj.depth_m = obj.depth_m
            new_obj.u_color = obj.u_color
            new_obj.v_color = obj.v_color
            new_obj.u_depth = obj.u_depth
            new_obj.v_depth = obj.v_depth
            new_obj.bbox_area = obj.bbox_area

            if not obj.reachable:
                # Profundidad invalida en camara: no hay punto util que transformar.
                new_obj.point.x = 0.0
                new_obj.point.y = 0.0
                new_obj.point.z = 0.0
                new_obj.reachable = False
                new_obj.reason = obj.reason or "invalid_depth"
                out_objects.append(new_obj)
                continue

            point_in = PointStamped()
            point_in.header.frame_id = source_frame
            point_in.header.stamp = msg.header.stamp
            point_in.point = obj.point

            try:
                point_out = do_transform_point(point_in, transform)
            except Exception as e:
                self.get_logger().error(f"Error transformando {obj.object_id}: {e}")
                new_obj.point.x = 0.0
                new_obj.point.y = 0.0
                new_obj.point.z = 0.0
                new_obj.reachable = False
                new_obj.reason = "tf_transform_failed"
                out_objects.append(new_obj)
                continue

            new_obj.point = point_out.point

            reachable, reason = self.workspace_check(
                point_out.point.x, point_out.point.y, point_out.point.z
            )
            new_obj.reachable = reachable
            new_obj.reason = reason
            out_objects.append(new_obj)

        out_arr = Object3DArray()
        out_arr.header.stamp = msg.header.stamp
        out_arr.header.frame_id = self.base_frame
        out_arr.objects = out_objects
        self.objects_base_pub.publish(out_arr)

        reachable_objects = [o for o in out_objects if o.reachable]
        if reachable_objects:
            selected = max(reachable_objects, key=lambda o: (o.bbox_area, o.confidence))
            selected_point = PointStamped()
            selected_point.header = out_arr.header
            selected_point.point = selected.point
            self.selected_base_pub.publish(selected_point)


def main(args=None):
    rclpy.init(args=args)
    node = CameraToBaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
