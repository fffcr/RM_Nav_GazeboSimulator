# Copyright 2025 Lihan Chen
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import subprocess
import tempfile

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString


def _bool_env(name: str, default: str = "false") -> str:
    """Read env var as lowercase string, fallback to default (for xacro mappings)."""
    return os.getenv(name, default).strip().lower()


def generate_launch_description():
    pkg_simulator = get_package_share_directory("rmu_gazebo_simulator")
    pkg_rmoss_gz_resources = get_package_share_directory("rmoss_gz_resources")

    # Path to custom robot URDF xacro entry point and model directory
    sentry_model_dir = os.path.join(
        pkg_rmoss_gz_resources, "resource", "models", "sentry"
    )
    robot_xacro_path = os.path.join(sentry_model_dir, "robot.urdf.xacro")
    bridge_config = os.path.join(pkg_simulator, "config", "ros_gz_bridge.yaml")
    robot_config = os.path.join(pkg_simulator, "config", "base_params.yaml")

    # Get spawn robot init pose from world config
    gz_world_path = os.path.join(pkg_simulator, "config", "gz_world.yaml")
    with open(gz_world_path) as file:
        config = yaml.safe_load(file)
        selected_world = config.get("world")
        robots = config["robots"].get(selected_world)

    ld = LaunchDescription()

    for robot in robots:
        # ----------------------------------------------------------------
        # Step 1: xacro → URDF string
        # ----------------------------------------------------------------
        xacro_doc = xacro.process_file(
            robot_xacro_path,
            mappings={"use_force_commands": _bool_env("FORCE_MODE", "false")},
        )
        robot_urdf_xml = xacro_doc.toprettyxml(indent="  ")

        # ----------------------------------------------------------------
        # Step 2: URDF → SDF string (via ign sdf -p, for Gazebo spawning)
        # ----------------------------------------------------------------
        # Write temp URDF into the model directory so relative mesh
        # paths (e.g. meshes/part_004_NAUO4.stl) are resolved by ign sdf -p.
        with tempfile.NamedTemporaryFile(
            mode="w", suffix=".urdf", delete=False,
            encoding="utf-8", dir=sentry_model_dir
        ) as f:
            f.write(robot_urdf_xml)
            urdf_tmp_path = f.name

        try:
            result = subprocess.run(
                ["ign", "sdf", "-p", urdf_tmp_path],
                capture_output=True,
                text=True,
                timeout=30,
            )
            if result.returncode != 0:
                raise RuntimeError(
                    f"ign sdf -p failed for {robot['name']}:\n{result.stderr}"
                )
            robot_sdf_xml = result.stdout
            # Convert relative mesh paths to model:// URIs so Gazebo can
            # resolve them via IGN_GAZEBO_RESOURCE_PATH.
            robot_sdf_xml = robot_sdf_xml.replace(
                '<uri>meshes/', '<uri>model://sentry/meshes/'
            )
        finally:
            os.unlink(urdf_tmp_path)

        # ----------------------------------------------------------------
        # Step 3: replace <robot_name> placeholder in bridge config
        # ----------------------------------------------------------------
        aft_replace_ros_bridge_params = ReplaceString(
            source_file=bridge_config,
            replacements={"<robot_name>": robot["name"]},
        )

        # ----------------------------------------------------------------
        # Step 4: spawn robot in Gazebo
        # ----------------------------------------------------------------
        spawn_robot = Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-string",
                robot_sdf_xml,
                "-name",
                robot["name"],
                "-allow_renaming",
                "true",
                "-x",
                robot["x_pose"],
                "-y",
                robot["y_pose"],
                "-z",
                robot["z_pose"],
                "-Y",
                robot["yaw"],
            ],
        )

        # ----------------------------------------------------------------
        # Step 5: rmua19_robot_base (gimbal + shooter + chassis IF)
        # ----------------------------------------------------------------
        robot_base = Node(
            package="rmoss_gz_base",
            executable="rmua19_robot_base",
            namespace=robot["name"],
            parameters=[robot_config, {"robot_name": robot["name"]}],
        )

        # ----------------------------------------------------------------
        # Step 6: robot_state_publisher (URDF → TF)
        # Publish TF to root /tf with sentry/ prefix so RViz can see them.
        # ----------------------------------------------------------------
        robot_state_publisher = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[
                {
                    "use_sim_time": True,
                    "robot_description": robot_urdf_xml,
                    "frame_prefix": f"{robot['name']}/",
                }
            ],
        )

        # ----------------------------------------------------------------
        # Step 7: ros_gz_bridge (sensor data, joint states, etc.)
        # ----------------------------------------------------------------
        robot_ign_bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            namespace=robot["name"],
            parameters=[{"config_file": aft_replace_ros_bridge_params}],
        )

        # ----------------------------------------------------------------
        # Step 8: register lidar sensor frame in TF tree
        # Gazebo ignores URDF <frame_name> and auto-generates:
        #   frame_id = "yaw_link/lidar_sensor" (parent_link/sensor_name)
        # robot_state_publisher only publishes yaw_link, so we bridge the gap.
        # ----------------------------------------------------------------
        static_tf_lidar = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                "0", "0", "0", "0", "0", "0",
                f"{robot['name']}/yaw_link",
                f"{robot['name']}/yaw_link/lidar_sensor",
            ],
        )

        # ----------------------------------------------------------------
        # Step 9: set_performer (level-of-detail optimization)
        # ----------------------------------------------------------------
        set_performer_service = ExecuteProcess(
            cmd=[
                "ign",
                "service",
                "-s",
                "/world/default/level/set_performer",
                "--reqtype",
                "ignition.msgs.StringMsg",
                "--reptype",
                "ignition.msgs.Boolean",
                "--timeout",
                "2000",
                "--req",
                f'data: "{robot["name"]}"',
            ],
            output="screen",
        )

        ld.add_action(spawn_robot)
        ld.add_action(robot_base)
        ld.add_action(robot_state_publisher)
        ld.add_action(robot_ign_bridge)
        ld.add_action(static_tf_lidar)
        ld.add_action(set_performer_service)

    return ld
