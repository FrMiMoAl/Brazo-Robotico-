#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
from std_msgs.msg import Float32, Int32
from sensor_msgs.msg import JointState

class ESP32ToRVizNode(Node):
    def __init__(self):
        super().__init__('esp32_to_rviz')

        # Joint States Publisher
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Subscriptions to ESP32 Topics
        self.sub_nema = self.create_subscription(Float32, '/motor_nema/current_deg', self.nema_callback, 10)
        self.sub_pololu = self.create_subscription(Float32, '/motor_pololu/current_deg', self.pololu_callback, 10)
        self.sub_servo1 = self.create_subscription(Int32, '/servo1/current_deg', self.servo1_callback, 10)
        self.sub_servo2 = self.create_subscription(Int32, '/servo2/current_deg', self.servo2_callback, 10)
        self.sub_servo3 = self.create_subscription(Int32, '/servo3/current_deg', self.servo3_callback, 10)

        # Nombres de las articulaciones del URDF brazo4central
        self.joint_names = [
            '1', '2', '3', '4',
            'left_joint', 'right_joint',
            'left_gear', 'right_gear_joint'
        ]

        # Current values in RViz space (radians)
        # Initialize to home: J1=0.0, J2=0.0, J3=0.0, J4=180 deg (pi rad)
        self.q_j1 = 0.0
        self.q_j2 = 0.0
        self.q_j3 = 0.0
        self.q_j4 = 0.0
        self.q_gripper = 0.0

        # Flag to wait until we receive any message from ESP32
        self.received_any = False

        # Timer to publish JointState at 50Hz (0.02s)
        self.timer = self.create_timer(0.02, self.timer_callback)

        self.get_logger().info("ESP32 to RViz Mapper Node Started.")
        self.get_logger().info("Waiting for data from ESP32 to start publishing to RViz...")

    def map_range(self, val, in_min, in_max, out_min, out_max):
        # Prevent division by zero and clamp input
        if in_max == in_min:
            return out_min
        val = max(min(in_min, in_max), min(max(in_min, in_max), val))
        return out_min + (val - in_min) * (out_max - out_min) / (in_max - in_min)

    def nema_callback(self, msg):
        self.received_any = True
        # NEMA (J1) goes from 0.0 to 130.0 deg -> maps to 0.0 to -130.0 deg in RViz
        j1_deg = self.map_range(msg.data, 0.0, 130.0, 0.0, -130.0)
        self.q_j1 = math.radians(j1_deg)

    def pololu_callback(self, msg):
        self.received_any = True
        # Pololu (J2) goes from -320.0 to 0.0 deg -> maps to -140.0 to 0.0 deg in RViz
        j2_deg = self.map_range(msg.data, -320.0, 0.0, -140.0, 0.0)
        self.q_j2 = math.radians(j2_deg)

    def servo1_callback(self, msg):
        self.received_any = True
        # Servo 1 (J3) goes from 0.0 to 180.0 deg -> maps to 0.0 to 150.0 deg in RViz
        j3_deg = self.map_range(float(msg.data), 0.0, 180.0, 0.0, 150.0)
        self.q_j3 = math.radians(j3_deg)

    def servo2_callback(self, msg):
        self.received_any = True
        # Servo 2 (J4) goes from 0.0 to 150.0 deg -> maps to 0.0 to 180.0 deg in RViz
        j4_deg = self.map_range(float(msg.data), 0.0, 150.0, 0.0, 180.0)
        self.q_j4 = math.radians(j4_deg)

    def servo3_callback(self, msg):
        self.received_any = True
        # Servo 3 (Gripper) goes from 0.0 to 180.0 deg -> maps to -0.2 to 0.6 rad in RViz
        self.q_gripper = self.map_range(float(msg.data), 0.0, 180.0, -0.2, 0.6)

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        
        # Populate the positions:
        # '1', '2', '3', '4', 'left_joint', 'right_joint', 'left_gear', 'right_gear_joint'
        msg.position = [
            self.q_j1,
            self.q_j2,
            self.q_j3,
            self.q_j4,
            self.q_gripper,            # left_joint
            self.q_gripper,            # right_joint
            0.41 * self.q_gripper,     # left_gear
            0.41 * self.q_gripper      # right_gear_joint
        ]
        
        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ESP32ToRVizNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
