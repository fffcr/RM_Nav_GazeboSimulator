"""Launch Ignition Gazebo with omni-wheel robot, spawn, and bridge ROS 2 ↔ Gazebo."""

import os

from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg_share = get_package_share_path('sim_environment')

    world_path     = os.path.join(pkg_share, 'world', 'corridor.sdf')
    urdf_path      = os.path.join(pkg_share, 'urdf', 'omni_robot', 'my_robot.urdf.xacro')
    bridge_yaml_path = os.path.join(pkg_share, 'config', 'bridge_topics.yaml')
    map_yaml_path  = os.path.join(pkg_share, 'map', 'corridor.yaml')

    robot_description = ParameterValue(
        Command(['xacro', ' ', urdf_path]),
        value_type=str,
    )

    # ---- 1. Gazebo server ----
    gz_server = ExecuteProcess(
        cmd=['ign', 'gazebo', world_path, '-r'],
        output='screen',
    )

    # ---- 2. robot_state_publisher ----
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    # ---- 3. 生成机器人 ----
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'my_robot',
            '-z', '0',
        ],
        output='screen',
    )

    # ---- 4. ros_gz_bridge（bridge_topics.yaml） ----
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_yaml_path}],
        remappings=[('/scan', '/scan_raw')],
        output='screen',
    )

    # ---- 5. 修复 scan frame_id（my_robot/base_footprint/lidar → camera_link） ----
    scan_fix = Node(
        package='sim_environment',
        executable='fix_scan_frame.py',
        name='fix_scan_frame',
        parameters=[{'frame_id': 'camera_link'}],
        output='screen',
    )

    # ---- 6. map_server ----
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{
            'yaml_filename': map_yaml_path,
            'use_sim_time': True,
            'frame_id': 'map',
        }],
        remappings=[('/map', '/map_static')],
        output='screen',
    )

    # ---- 6b. map_relay（/map_static → /map，1 Hz） ----
    map_relay = Node(
        package='sim_environment',
        executable='map_relay.py',
        name='map_relay',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # ---- 7. RViz ----
    rviz = ExecuteProcess(
        cmd=['ros2', 'run', 'rviz2', 'rviz2', '--ros-args', '-p', 'use_sim_time:=true'],
        output='screen',
    )

    # ---- 8. 自动激活 map_server ----
    map_activate = ExecuteProcess(
        cmd=['bash', '-c',
             'while ! ros2 topic echo /clock --once 2>/dev/null; '
             'do sleep 1; done; '
             'sleep 5; '
             'ros2 lifecycle set /map_server configure && '
             'ros2 lifecycle set /map_server activate'],
        output='screen',
    )

    # ---- 9. map -> odom 静态 TF ----
    map_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )

    # ---- 9b. odom -> base_footprint TF（Gazebo 真值位姿） ----
    odom_to_tf = Node(
        package='sim_environment',
        executable='odom_to_tf.py',
        name='odom_to_tf',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # ---- 10. cmd_vel 方向可视化 ----
    cmd_vel_viz = Node(
        package='sim_environment',
        executable='cmd_vel_visualizer.py',
        name='cmd_vel_visualizer',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # ---- 启动顺序 ----
    return LaunchDescription([
        gz_server,
        robot_state_pub,
        map_server,
        map_relay,
        map_odom_tf,
        rviz,
        map_activate,
        RegisterEventHandler(
            OnProcessStart(
                target_action=robot_state_pub,
                on_start=[spawn_robot, bridge, scan_fix, odom_to_tf, cmd_vel_viz],
            )
        ),
    ])
