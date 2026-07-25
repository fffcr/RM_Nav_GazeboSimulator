#!/usr/bin/env python3
"""监控障碍物(cylinder)位姿变化时对应的 cmd_vel 输出。

监听 /gz_scene_poses 中名字含 "cylinder" 的物体，当其位姿相对 map
发生显著变化（位移 > 0.01m）时，打印位移前/后两帧的 cmd_vel。
"""

import math
from collections import deque
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Twist


class ObstacleCmdMonitor(Node):
    def __init__(self):
        super().__init__('obstacle_cmd_monitor')

        # 缓存每个 cylinder 的上次位姿，用于检测变化
        self._prev_poses = {}          # child_frame_id -> (x, y, yaw)
        self._move_threshold = self.declare_parameter(
            'move_threshold', 0.01).value  # 位移阈值 (m)

        # cmd_vel 历史: 保留最近 5 帧，便于位移发生时找到"前一帧"
        self._cmd_history = deque(maxlen=5)       # (stamp_sec, Twist)
        self._cmd_sim_history = deque(maxlen=5)   # (stamp_sec, Twist)

        # 订阅场景真值位姿 (含所有模型包括障碍物)
        self.create_subscription(
            TFMessage, '/gz_scene_poses', self.scene_callback, 10)

        # 订阅规划器输出的速度指令
        self.create_subscription(
            Twist, '/neupan_cmd_vel', self.cmd_callback, 10)

        # 同时也看 /cmd_vel（实际发给仿真器的）
        self.create_subscription(
            Twist, '/cmd_vel', self.cmd_sim_callback, 10)

        # 每秒汇总一次
        self._cyl_count = 0
        self._cmd_count = 0
        self.create_timer(1.0, self.heartbeat)

        self.get_logger().info(
            '监控已启动: 监听 cylinder 位姿变化 + /neupan_cmd_vel + /cmd_vel')

    def heartbeat(self):
        self.get_logger().info(
            f'心跳: 收到 {self._cyl_count} 条场景位姿, '
            f'{self._cmd_count} 条 cmd_vel (最近1秒)',
            throttle_duration_sec=1)
        self._cyl_count = 0
        self._cmd_count = 0

    def quat_to_yaw(self, q):
        """四元数 → yaw"""
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.z * q.z + q.y * q.y)
        return math.atan2(siny, cosy)

    # ---------- cmd_vel 订阅 ----------
    def cmd_callback(self, msg: Twist):
        self._cmd_count += 1
        now = self.get_clock().now().nanoseconds / 1e9
        self._cmd_history.append((now, msg))

    def cmd_sim_callback(self, msg: Twist):
        now = self.get_clock().now().nanoseconds / 1e9
        self._cmd_sim_history.append((now, msg))

    # ---------- 场景位姿订阅 ----------
    def scene_callback(self, msg: TFMessage):
        self._cyl_count += 1
        for t in msg.transforms:
            name = t.child_frame_id
            # 只看障碍物 (cylinder / obstacle / box 等)
            if 'cyl' not in name.lower() and 'obs' not in name.lower():
                continue

            x = t.transform.translation.x
            y = t.transform.translation.y
            yaw = self.quat_to_yaw(t.transform.rotation)

            prev = self._prev_poses.get(name)
            if prev is not None:
                dx = x - prev[0]
                dy = y - prev[1]
                dist = math.hypot(dx, dy)
                if dist >= self._move_threshold:
                    self._log_move(name, prev, (x, y, yaw), dist)
            else:
                # 首次发现该障碍物
                self.get_logger().info(
                    f'[发现] {name}: pos=({x:.3f}, {y:.3f}, yaw={yaw:.3f})')

            self._prev_poses[name] = (x, y, yaw)

    # ---------- 格式化输出 ----------
    @staticmethod
    def _fmt_cmd(label: str, entry, now: float):
        """格式化一条 cmd_vel 记录"""
        if entry is None:
            return f'  {label}: (无记录)'
        ts, twist = entry
        age_ms = (now - ts) * 1000.0
        return (f'  {label}: vx={twist.linear.x:.4f}, vy={twist.linear.y:.4f}, '
                f'vz={twist.angular.z:.4f}  (t={ts:.3f}, {age_ms:.0f}ms 前)')

    def _log_move(self, name, prev, cur, dist):
        px, py, pyaw = prev
        cx, cy, cyaw = cur
        now = self.get_clock().now().nanoseconds / 1e9

        # 从历史队列里找"前一帧"和"当前帧"
        def pick_two(history):
            if len(history) == 0:
                return None, None
            if len(history) == 1:
                return None, history[-1]
            return history[-2], history[-1]

        prev_neupan, cur_neupan = pick_two(self._cmd_history)
        prev_sim, cur_sim = pick_two(self._cmd_sim_history)

        lines = [
            '========================================',
            f'[障碍物移动] {name}',
            f'  旧位姿: ({px:.3f}, {py:.3f}, yaw={pyaw:.3f})',
            f'  新位姿: ({cx:.3f}, {cy:.3f}, yaw={cyaw:.3f})',
            f'  位移量: {dist:.3f} m',
            f'  --- /neupan_cmd_vel ---',
            self._fmt_cmd('位移前(前一帧)', prev_neupan, now),
            self._fmt_cmd('位移后(当前帧)', cur_neupan, now),
            f'  --- /cmd_vel ---',
            self._fmt_cmd('位移前(前一帧)', prev_sim, now),
            self._fmt_cmd('位移后(当前帧)', cur_sim, now),
            '========================================',
        ]
        self.get_logger().warn('\n'.join(lines))


def main():
    rclpy.init()
    rclpy.spin(ObstacleCmdMonitor())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
