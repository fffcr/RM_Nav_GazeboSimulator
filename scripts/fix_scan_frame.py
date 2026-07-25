#!/usr/bin/env python3
"""Relay LaserScan messages with a corrected frame_id.

Bridge publishes to /scan_raw (with wrong frame_id like my_robot/base_footprint/lidar).
This node republishes to /scan with frame_id set to the configured value (e.g. camera_link).
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class FixScanFrame(Node):
    def __init__(self):
        super().__init__('fix_scan_frame')
        self.declare_parameter('frame_id', 'camera_link')
        self.frame_id = self.get_parameter('frame_id').value

        self.pub = self.create_publisher(LaserScan, '/scan', 10)
        self.sub = self.create_subscription(
            LaserScan, '/scan_raw', self.callback, 10)

        self.get_logger().info(f'Relaying /scan_raw → /scan (frame_id={self.frame_id})')

    def callback(self, msg: LaserScan):
        msg.header.frame_id = self.frame_id
        self.pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(FixScanFrame())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
