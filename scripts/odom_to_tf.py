#!/usr/bin/env python3
"""Publish odom -> base_footprint TF from Gazebo ground-truth scene poses.

Subscribes to /gz_scene_poses (bridged from /world/empty/pose/info),
caches the latest my_robot pose, and republishes it at 50 Hz as
odom -> base_footprint.  High-frequency republishing prevents RViz
"queue full" errors caused by TF timestamp mismatches with LaserScan.
"""

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomToTF(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        self.tf_broadcaster = TransformBroadcaster(self)
        self._latest = None   # 缓存最新真值位姿
        self._msg_count = 0

        self.sub = self.create_subscription(
            TFMessage, '/gz_scene_poses', self.callback, 10)

        # 50 Hz 高频重发，确保 tf2 在任何时间戳都能查到 odom -> base_footprint
        self.timer = self.create_timer(1.0 / 50.0, self.publish_cached)

        self.get_logger().info(
            'Caching my_robot pose from /gz_scene_poses, '
            'republishing odom -> base_footprint @ 50 Hz')

    def callback(self, msg: TFMessage):
        for t in msg.transforms:
            if t.child_frame_id == 'my_robot':
                self._latest = t
                return

    def publish_cached(self):
        if self._latest is None:
            return

        t = self._latest
        out = TransformStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'odom'
        out.child_frame_id = 'base_footprint'
        out.transform = t.transform
        self.tf_broadcaster.sendTransform(out)

        self._msg_count += 1
        if self._msg_count % 50 == 0:
            self.get_logger().info(
                f'#{self._msg_count} odom->base_footprint: '
                f'x={out.transform.translation.x:.3f}, '
                f'y={out.transform.translation.y:.3f}')


def main():
    rclpy.init()
    rclpy.spin(OdomToTF())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
