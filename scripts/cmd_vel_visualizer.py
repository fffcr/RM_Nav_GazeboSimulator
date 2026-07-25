#!/usr/bin/env python3
"""Visualize /cmd_vel as an arrow marker in RViz."""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, Vector3
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA


class CmdVelVisualizer(Node):
    def __init__(self):
        super().__init__('cmd_vel_visualizer')
        self.declare_parameter('frame_id', 'base_footprint')
        self.declare_parameter('scale', 0.5)  # arrow length per m/s
        self.frame_id = self.get_parameter('frame_id').value
        self.scale = self.get_parameter('scale').value

        self.pub = self.create_publisher(Marker, '/cmd_vel_marker', 10)
        self.sub = self.create_subscription(
            Twist, '/cmd_vel', self.callback, 10)
        self.get_logger().info(
            f'Visualizing /cmd_vel as arrow in {self.frame_id} frame')

    def callback(self, msg: Twist):
        speed = math.hypot(msg.linear.x, msg.linear.y)
        yaw = math.atan2(msg.linear.y, msg.linear.x)

        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'cmd_vel'
        m.id = 0
        m.type = Marker.ARROW
        m.action = Marker.ADD

        # arrow shaft
        m.points = [
            Point(x=0.0, y=0.0, z=0.1),
            Point(x=self.scale * msg.linear.x,
                  y=self.scale * msg.linear.y,
                  z=0.1),
        ]
        m.scale = Vector3(x=0.05, y=0.1, z=0.1)  # shaft width, head width, head length

        # color: green=forward, red=stop
        m.color = ColorRGBA(r=1.0, g=1.0 - min(speed, 1.0), b=0.0, a=0.8)
        m.lifetime.sec = 0  # persist until next update
        self.pub.publish(m)


def main():
    rclpy.init()
    rclpy.spin(CmdVelVisualizer())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
