#!/usr/bin/env python3
"""
===============================================================================
MyroNode ROS 2 Bridge Node
Seeed Studio XIAO STM32C5: micro-ROS Compatible Serial <-> ROS 2 Topics
===============================================================================
Publishes:
  - /node2/imu                 (sensor_msgs/msg/Imu)
  - /node2/pose                (geometry_msgs/msg/PoseStamped)
  - /node2/tilt                (geometry_msgs/msg/Vector3Stamped)
  - /node1/servo_state         (sensor_msgs/msg/JointState)
  - /node1/pose                (geometry_msgs/msg/PoseStamped)
  - /visualization_marker_array (visualization_msgs/msg/MarkerArray)
  - /tf                        (tf2_msgs/msg/TFMessage via TransformBroadcaster)

Subscribes:
  - /servo/cmd                 (std_msgs/msg/Float32)  -> Angle in degrees [0 to 300]
  - /mimic/mode                (std_msgs/msg/Int32)    -> 0: Mimic, 1: Manual, 2: Center
===============================================================================
"""

import sys
import os
import json
import math
import time
import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Vector3Stamped, PoseStamped, TransformStamped
from std_msgs.msg import Float32, Int32
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster

def euler_to_quaternion(roll_rad, pitch_rad, yaw_rad=0.0):
    cy = math.cos(yaw_rad * 0.5)
    sy = math.sin(yaw_rad * 0.5)
    cp = math.cos(pitch_rad * 0.5)
    sp = math.sin(pitch_rad * 0.5)
    cr = math.cos(roll_rad * 0.5)
    sr = math.sin(roll_rad * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw

class MyroNodeBridge(Node):
    def __init__(self):
        super().__init__('myronode_bridge')

        # Default port: /dev/ttyS21 on Linux/WSL1 (COM21 on Windows)
        default_port = '/dev/ttyS21' if os.name != 'nt' else 'COM21'
        self.declare_parameter('port', default_port)
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baudrate').get_parameter_value().integer_value

        self.get_logger().info(f'Opening connection to MyroNode Gateway on {port} @ {baud} baud...')

        try:
            self.ser = serial.Serial(port, baud, timeout=0.05)
            self.get_logger().info(f'Successfully connected to serial device on {port}')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port {port}: {e}')
            self.get_logger().warn('Ensure COM21 / /dev/ttyS21 is not occupied by another program (e.g. dashboard.py)')
            self.ser = None

        # Publishers
        self.pub_imu = self.create_publisher(Imu, '/node2/imu', 10)
        self.pub_pose_imu = self.create_publisher(PoseStamped, '/node2/pose', 10)
        self.pub_tilt = self.create_publisher(Vector3Stamped, '/node2/tilt', 10)
        self.pub_joint = self.create_publisher(JointState, '/node1/servo_state', 10)
        self.pub_pose_servo = self.create_publisher(PoseStamped, '/node1/pose', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)

        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Command Subscribers
        self.sub_cmd = self.create_subscription(Float32, '/servo/cmd', self.cmd_callback, 10)
        self.sub_mode = self.create_subscription(Int32, '/mimic/mode', self.mode_callback, 10)

        # 50 Hz poll timer
        self.timer = self.create_timer(0.02, self.poll_serial)
        self.buffer = ""

    def cmd_callback(self, msg: Float32):
        if self.ser and self.ser.is_open:
            cmd = f"TARGET:{msg.data:.1f}\r\n"
            self.ser.write(cmd.encode('utf-8'))
            self.get_logger().info(f'Sent servo target: {msg.data:.1f}°')

    def mode_callback(self, msg: Int32):
        if self.ser and self.ser.is_open:
            if msg.data == 0:
                self.ser.write(b"MIMIC\r\n")
            elif msg.data == 1:
                self.ser.write(b"MANUAL\r\n")
            elif msg.data == 2:
                self.ser.write(b"HOLD\r\n")
            self.get_logger().info(f'Switched operating mode to: {msg.data}')

    def poll_serial(self):
        if not self.ser or not self.ser.is_open:
            return

        try:
            available = self.ser.in_waiting
            if available > 0:
                data = self.ser.read(available)
                if data:
                    self.buffer += data.decode('utf-8', errors='ignore')
                    while '\n' in self.buffer:
                        line, self.buffer = self.buffer.split('\n', 1)
                        line = line.strip()
                        if line.startswith('{') and line.endswith('}'):
                            self.process_json(line)
        except Exception as e:
            self.get_logger().warn(f'Serial exception: {e}')

    def process_json(self, line: str):
        try:
            pkt = json.loads(line)
            now = self.get_clock().now().to_msg()
            markers = []

            # =========================================================================
            # 1. NODE 2: IMU Processing (Euler -> Quaternion & TF)
            # =========================================================================
            if 'imu' in pkt:
                imu = pkt['imu']
                roll_deg = float(imu.get('roll', 0.0))
                pitch_deg = float(imu.get('pitch', 0.0))
                roll_rad = math.radians(roll_deg)
                pitch_rad = math.radians(pitch_deg)

                qx, qy, qz, qw = euler_to_quaternion(roll_rad, pitch_rad, 0.0)

                # A. sensor_msgs/msg/Imu
                imu_msg = Imu()
                imu_msg.header.stamp = now
                imu_msg.header.frame_id = 'node2_imu'
                imu_msg.orientation.x = qx
                imu_msg.orientation.y = qy
                imu_msg.orientation.z = qz
                imu_msg.orientation.w = qw
                # Angular rate (gz dps -> rad/s)
                imu_msg.angular_velocity.x = float(imu.get('gx', 0.0)) * (math.pi / 180.0)
                imu_msg.angular_velocity.y = float(imu.get('gy', 0.0)) * (math.pi / 180.0)
                imu_msg.angular_velocity.z = float(imu.get('gz', 0.0)) * (math.pi / 180.0)
                # Acceleration (g -> m/s^2)
                imu_msg.linear_acceleration.x = float(imu.get('ax', 0.0)) * 9.80665
                imu_msg.linear_acceleration.y = float(imu.get('ay', 0.0)) * 9.80665
                imu_msg.linear_acceleration.z = float(imu.get('az', 1.0)) * 9.80665
                self.pub_imu.publish(imu_msg)

                # B. geometry_msgs/msg/PoseStamped
                pose_msg = PoseStamped()
                pose_msg.header.stamp = now
                pose_msg.header.frame_id = 'world'
                pose_msg.pose.position.x = -0.3
                pose_msg.pose.position.y = 0.0
                pose_msg.pose.position.z = 0.1
                pose_msg.pose.orientation = imu_msg.orientation
                self.pub_pose_imu.publish(pose_msg)

                # C. Vector3Stamped (Raw tilt degrees)
                tilt_msg = Vector3Stamped()
                tilt_msg.header.stamp = now
                tilt_msg.header.frame_id = 'node2_imu'
                tilt_msg.vector.x = roll_deg
                tilt_msg.vector.y = pitch_deg
                tilt_msg.vector.z = float(imu.get('gz', 0.0))
                self.pub_tilt.publish(tilt_msg)

                # D. Broadcast TF: world -> node2_imu
                t_imu = TransformStamped()
                t_imu.header.stamp = now
                t_imu.header.frame_id = 'world'
                t_imu.child_frame_id = 'node2_imu'
                t_imu.transform.translation.x = -0.3
                t_imu.transform.translation.y = 0.0
                t_imu.transform.translation.z = 0.1
                t_imu.transform.rotation = imu_msg.orientation
                self.tf_broadcaster.sendTransform(t_imu)

                # E. RViz2 3D Marker: Node 2 XIAO Board (Cyan)
                m_board = Marker()
                m_board.header.stamp = now
                m_board.header.frame_id = 'node2_imu'
                m_board.ns = 'node2'
                m_board.id = 0
                m_board.type = Marker.CUBE
                m_board.action = Marker.ADD
                m_board.scale.x = 0.21  # 21mm length
                m_board.scale.y = 0.17  # 17.5mm width
                m_board.scale.z = 0.02
                m_board.color.r = 0.02
                m_board.color.g = 0.71
                m_board.color.b = 0.83  # Cyan
                m_board.color.a = 0.95
                markers.append(m_board)

            # =========================================================================
            # 2. NODE 1: Servo Actuator Processing
            # =========================================================================
            if 'servo' in pkt:
                servo = pkt['servo']
                angle_deg = float(servo.get('angle', 150.0))
                # Map 0..300 deg to -150..+150 deg (-2.618 to +2.618 rad)
                angle_rad = (angle_deg - 150.0) * (math.pi / 180.0)

                # A. sensor_msgs/msg/JointState
                joint_msg = JointState()
                joint_msg.header.stamp = now
                joint_msg.header.frame_id = 'world'
                joint_msg.name = ['scs0009_joint']
                joint_msg.position = [angle_rad]
                joint_msg.velocity = [0.0]
                joint_msg.effort = [float(servo.get('pos', 512))]
                self.pub_joint.publish(joint_msg)

                # B. Broadcast TF: world -> node1_base
                t_base = TransformStamped()
                t_base.header.stamp = now
                t_base.header.frame_id = 'world'
                t_base.child_frame_id = 'node1_base'
                t_base.transform.translation.x = 0.3
                t_base.transform.translation.y = 0.0
                t_base.transform.translation.z = 0.05
                t_base.transform.rotation.w = 1.0
                self.tf_broadcaster.sendTransform(t_base)

                # C. Broadcast TF: node1_base -> node1_horn (Rotating by servo angle)
                sqx, sqy, sqz, sqw = euler_to_quaternion(0.0, 0.0, angle_rad)
                t_horn = TransformStamped()
                t_horn.header.stamp = now
                t_horn.header.frame_id = 'node1_base'
                t_horn.child_frame_id = 'node1_horn'
                t_horn.transform.translation.z = 0.05
                t_horn.transform.rotation.x = sqx
                t_horn.transform.rotation.y = sqy
                t_horn.transform.rotation.z = sqz
                t_horn.transform.rotation.w = sqw
                self.tf_broadcaster.sendTransform(t_horn)

                # D. RViz2 3D Marker: Servo Body (Dark Slate)
                m_body = Marker()
                m_body.header.stamp = now
                m_body.header.frame_id = 'node1_base'
                m_body.ns = 'node1'
                m_body.id = 1
                m_body.type = Marker.CUBE
                m_body.action = Marker.ADD
                m_body.scale.x = 0.12
                m_body.scale.y = 0.08
                m_body.scale.z = 0.10
                m_body.color.r = 0.15
                m_body.color.g = 0.20
                m_body.color.b = 0.28
                m_body.color.a = 0.95
                markers.append(m_body)

                # E. RViz2 3D Marker: Rotating Servo Arm / Horn (Emerald)
                m_horn = Marker()
                m_horn.header.stamp = now
                m_horn.header.frame_id = 'node1_horn'
                m_horn.ns = 'node1'
                m_horn.id = 2
                m_horn.type = Marker.ARROW
                m_horn.action = Marker.ADD
                m_horn.scale.x = 0.18  # Length
                m_horn.scale.y = 0.03  # Width
                m_horn.scale.z = 0.03  # Height
                m_horn.color.r = 0.06
                m_horn.color.g = 0.72
                m_horn.color.b = 0.50  # Emerald
                m_horn.color.a = 1.0
                markers.append(m_horn)

            if markers:
                ma = MarkerArray()
                ma.markers = markers
                self.pub_markers.publish(ma)

        except Exception as e:
            pass

def main(args=None):
    rclpy.init(args=args)
    bridge = MyroNodeBridge()
    try:
        rclpy.spin(bridge)
    except Exception:
        pass
    finally:
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
