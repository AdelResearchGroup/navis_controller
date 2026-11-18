"""Start both Navis bridge nodes (HTTP and websocket).

Usage:
    ros2 launch navis_controller navis.launch.py
    ros2 launch navis_controller navis.launch.py robot_ip:=192.168.6.102 namespace:=bunker
    ros2 launch navis_controller navis.launch.py params_file:=/path/to/my_navis.yaml

The robot address comes from the launch arguments (robot_ip, http_port, ws_port) and overrides
base_url / ws_url in the params file. All other node parameters are set in params_file.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    default_params = os.path.join(get_package_share_directory("navis_controller"), "config", "navis.yaml")

    robot_ip = LaunchConfiguration("robot_ip")
    http_port = LaunchConfiguration("http_port")
    ws_port = LaunchConfiguration("ws_port")
    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("params_file")
    log_level = LaunchConfiguration("log_level")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_ip", default_value="192.168.6.102", description="IP address of the Navis module"
            ),
            DeclareLaunchArgument("http_port", default_value="8880", description="Navis HTTP API port"),
            DeclareLaunchArgument("ws_port", default_value="9090", description="Navis websocket API port"),
            DeclareLaunchArgument("namespace", default_value="", description="Namespace for both nodes"),
            DeclareLaunchArgument("params_file", default_value=default_params, description="Node parameters YAML"),
            DeclareLaunchArgument("log_level", default_value="info", description="Log level for both nodes"),
            Node(
                package="navis_controller",
                executable="navis_http",
                name="navis_http",
                namespace=namespace,
                output="screen",
                emulate_tty=True,
                parameters=[params_file, {"base_url": ["http://", robot_ip, ":", http_port]}],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="navis_controller",
                executable="navis_websocket",
                name="navis_websocket",
                namespace=namespace,
                output="screen",
                emulate_tty=True,
                parameters=[params_file, {"ws_url": ["ws://", robot_ip, ":", ws_port]}],
                arguments=["--ros-args", "--log-level", log_level],
            ),
        ]
    )
