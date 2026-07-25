#!/usr/bin/env python3
"""Relay /map_static → /map with fresh timestamps.  TRANSIENT_LOCAL both ends."""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid

LATCHED = QoSProfile(depth=1,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)

class MapRelay(Node):
    def __init__(self):
        super().__init__('map_relay')
        self.msg = None
        # 上游 map_server 用 TRANSIENT_LOCAL，这里必须匹配才能收到 latched 消息
        self.sub = self.create_subscription(OccupancyGrid, '/map_static',
                                            self.cb, LATCHED)
        # 下游 astar / RViz 也用 TRANSIENT_LOCAL 订阅，发布端也匹配
        self.pub = self.create_publisher(OccupancyGrid, '/map', LATCHED)
        self.timer = self.create_timer(1.0, self.tick)
        self.get_logger().info('map_relay ready')

    def cb(self, m):
        self.msg = m
        self.get_logger().info('map relay: received /map_static')

    def tick(self):
        if self.msg is not None:
            self.msg.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(self.msg)

def main():
    rclpy.init()
    rclpy.spin(MapRelay())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
