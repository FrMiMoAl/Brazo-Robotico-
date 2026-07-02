#!/usr/bin/env python3
import sys
import select
import termios
import tty
import threading
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from geometry_msgs.msg import Point

# Control settings guide/banner
BANNER = """
====================================================
     ROS 2 ROBOTIC ARM TELEOPERATION DASHBOARD      
====================================================
"""

class TeleopBrazoNode(Node):
    def __init__(self):
        super().__init__('teleop_brazo_node')

        # Publishers
        self.joint_pub = self.create_publisher(JointState, '/joint_commands', 10)
        self.gripper_pub = self.create_publisher(Bool, '/gripper_command', 10)
        self.cartesian_pub = self.create_publisher(Point, '/target_position', 10)

        # Internal state
        # 4-DOF Joint angles (J1, J2, J3, J4) in radians
        self.joints = [0.0, 0.0, 0.0, 0.0]
        # Cartesian position (x, y, z) in meters
        self.cartesian = [0.0, 0.0, 0.0]
        # Gripper state: True = Open, False = Closed
        self.gripper = True

        # Currently selected motor (0=J1, 1=J2, 2=J3, 3=J4)
        self.selected_motor = 0
        self.motor_names = ['J1 (NEMA)', 'J2 (Pololu)', 'J3 (Servo1)', 'J4 (Servo2)']

        # Increments (step sizes)
        self.joint_step = 0.05       # radians (~2.8 degrees)
        self.cartesian_step = 0.01   # meters (1 cm)

        # Limits (optional safety check, can be modified by user)
        self.joint_limits = [
            (-math.pi, math.pi),     # J1
            (-math.pi/2, math.pi/2), # J2
            (-math.pi/2, math.pi/2), # J3
            (-math.pi, math.pi)      # J4
        ]

        self.get_logger().info("Teleop node initialized and ready.")

    def publish_joints(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['J1', 'J2', 'J3', 'J4']
        msg.position = [float(val) for val in self.joints]
        self.joint_pub.publish(msg)

    def publish_gripper(self):
        msg = Bool()
        msg.data = bool(self.gripper)
        self.gripper_pub.publish(msg)

    def publish_cartesian(self):
        msg = Point()
        msg.x = float(self.cartesian[0])
        msg.y = float(self.cartesian[1])
        msg.z = float(self.cartesian[2])
        self.cartesian_pub.publish(msg)

    def reset_positions(self):
        self.joints = [0.0, 0.0, 0.0, 0.0]
        self.cartesian = [0.0, 0.0, 0.0]
        self.publish_joints()
        self.publish_cartesian()

    def process_key(self, key):
        # Motor selection: P cycles through motors
        if key == 'p' or key == 'P':
            self.selected_motor = (self.selected_motor + 1) % len(self.joints)

        # W/S: gradual movement on the currently selected motor
        elif key == 'w' or key == 'W':
            i = self.selected_motor
            self.joints[i] = min(self.joints[i] + self.joint_step, self.joint_limits[i][1])
            self.publish_joints()
        elif key == 's' or key == 'S':
            i = self.selected_motor
            self.joints[i] = max(self.joints[i] - self.joint_step, self.joint_limits[i][0])
            self.publish_joints()

        # Cartesian controls
        # X
        elif key == 't':
            self.cartesian[0] += self.cartesian_step
            self.publish_cartesian()
        elif key == 'g':
            self.cartesian[0] -= self.cartesian_step
            self.publish_cartesian()
        # Y
        elif key == 'y':
            self.cartesian[1] += self.cartesian_step
            self.publish_cartesian()
        elif key == 'h':
            self.cartesian[1] -= self.cartesian_step
            self.publish_cartesian()
        # Z
        elif key == 'u':
            self.cartesian[2] += self.cartesian_step
            self.publish_cartesian()
        elif key == 'j':
            self.cartesian[2] -= self.cartesian_step
            self.publish_cartesian()

        # Gripper controls
        elif key == 'o':
            self.gripper = True
            self.publish_gripper()
        elif key == 'c':
            self.gripper = False
            self.publish_gripper()

        # Adjustment of step sizes
        elif key == 'v':
            self.joint_step = min(self.joint_step + 0.01, 0.5)
        elif key == 'b':
            self.joint_step = max(self.joint_step - 0.01, 0.01)
        elif key == 'n':
            self.cartesian_step = min(self.cartesian_step + 0.005, 0.1)
        elif key == 'm':
            self.cartesian_step = max(self.cartesian_step - 0.005, 0.002)

        # Reset
        elif key == 'x':
            self.reset_positions()

    def print_dashboard(self):
        # ANSI Escape codes: clear screen and move cursor to top-left
        sys.stdout.write("\033[H\033[J")
        sys.stdout.write("\033[1;36m====================================================\033[0m\n")
        sys.stdout.write("\033[1;36m     ROS 2 ROBOTIC ARM TELEOPERATION DASHBOARD      \033[0m\n")
        sys.stdout.write("\033[1;36m====================================================\033[0m\n\n")

        # Arm Status Section
        sys.stdout.write("\033[1;33m--- Arm Status ---\033[0m\n")
        sys.stdout.write("  \033[1mJoint Positions:\033[0m\n")
        for i, val in enumerate(self.joints):
            deg = math.degrees(val)
            sys.stdout.write(f"    \033[1;32mJ{i+1}:\033[0m {val:6.3f} rad ({deg:6.1f}°)\n")

        sys.stdout.write(f"\n  \033[1mCartesian Target (IK):\033[0m\n")
        sys.stdout.write(f"    \033[1;32mX:\033[0m {self.cartesian[0]:6.3f} m\n")
        sys.stdout.write(f"    \033[1;32mY:\033[0m {self.cartesian[1]:6.3f} m\n")
        sys.stdout.write(f"    \033[1;32mZ:\033[0m {self.cartesian[2]:6.3f} m\n")

        grip_str = "\033[1;32mOPEN (True)\033[0m" if self.gripper else "\033[1;31mCLOSED (False)\033[0m"
        sys.stdout.write(f"\n  \033[1mGripper State:\033[0m {grip_str}\n\n")

        # Step Settings Section
        sys.stdout.write("\033[1;33m--- Step Sizes & Settings ---\033[0m\n")
        sys.stdout.write(f"  \033[1mJoint Step size:\033[0m     {self.joint_step:.3f} rad ({math.degrees(self.joint_step):.1f}°)\n")
        sys.stdout.write(f"  \033[1mCartesian Step size:\033[0m {self.cartesian_step:.3f} m\n\n")

        # Selected Motor Indicator
        sys.stdout.write("\033[1;33m--- Motor Seleccionado ---\033[0m\n")
        for i in range(len(self.joints)):
            marker = " >> " if i == self.selected_motor else "    "
            color = "\033[1;35m" if i == self.selected_motor else "\033[0m"
            deg = math.degrees(self.joints[i])
            sys.stdout.write(f"  {color}{marker}{self.motor_names[i]}: {self.joints[i]:6.3f} rad ({deg:6.1f}°)\033[0m\n")
        sys.stdout.write("\n")

        # Control Mappings Section
        sys.stdout.write("\033[1;33m--- Keyboard Mappings ---\033[0m\n")
        sys.stdout.write("  \033[1mMotor Control:\033[0m             \033[1mCartesian Commands:\033[0m\n")
        sys.stdout.write("    w  : mover motor (+)       X: t (+) / g (-)\n")
        sys.stdout.write("    s  : mover motor (-)       Y: y (+) / h (-)\n")
        sys.stdout.write("    p  : cambiar motor         Z: u (+) / j (-)\n\n")
        sys.stdout.write("  \033[1mGripper:\033[0m  o (Open) / c (Close)\n")
        sys.stdout.write("  \033[1mSteps:\033[0m    v/b (Joint Step +/-) | n/m (Cartesian Step +/-)\n")
        sys.stdout.write("  \033[1mUtility:\033[0m  x (Reset positions)  | Ctrl+C (Exit)\n\n")
        sys.stdout.flush()

def get_key(settings, timeout=0.1):
    # Set stdin to raw mode to read keypresses instantly
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    # Restore terminal settings
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main(args=None):
    rclpy.init(args=args)
    node = TeleopBrazoNode()

    # Spin ROS 2 callbacks in a background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Save original terminal settings
    settings = termios.tcgetattr(sys.stdin)

    try:
        node.print_dashboard()
        while rclpy.ok():
            key = get_key(settings, timeout=0.1)
            if key == '\x03':  # Ctrl+C
                break
            elif key:
                node.process_key(key)
                node.print_dashboard()
    except Exception as e:
        # We print to stderr since stdout might be corrupted in raw mode
        sys.stderr.write(f"\nError in teleop loop: {e}\n")
    finally:
        # ALWAYS restore original terminal settings on exit
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print("\nRestoring terminal configuration. Exiting teleop node...")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
