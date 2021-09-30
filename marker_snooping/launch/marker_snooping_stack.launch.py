from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

from launch.actions.execute_process import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hal_allied_vision_camera',
            executable='av_node',
            name='hal_allied_vision_camera',
            parameters=[os.path.join(get_package_share_directory("hal_allied_vision_camera"), 'params', 'params.yaml')],
        ),
        Node(
            package='hal_flir_d46',
            executable='hal_flir_d46',
            name='hal_flir_d46',
            output={
                    "stdout": "screen",
                    "stderr": "screen",
            },
            parameters=[os.path.join(get_package_share_directory("hal_flir_d46"), 'params', 'params.yaml')],
        ),
        Node(
            package='camera_target_tracking',
            executable='aruco_detector',
            name='aruco_detector',
            output={
                    "stdout": "screen",
                    "stderr": "screen",
            },
            parameters=[os.path.join(get_package_share_directory("camera_target_tracking"), 'params', 'params.yaml')],
        ),
        Node(
            package='marker_snooping',
            executable='marker_snooping',
            name='marker_snooping',
            output={
                    "stdout": "screen",
                    "stderr": "screen",
            },
            parameters=[os.path.join(get_package_share_directory("marker_snooping"), 'params', 'params.yaml')],
        ),
        Node(
            package='pasqua_tf',
            executable='camera_to_ptu_base',
            name='camera_to_ptu_base',
            output={
                    "stdout": "screen",
                    "stderr": "screen",
            },
            parameters=[os.path.join(get_package_share_directory("pasqua_tf"), 'params', 'camera_to_ptu_base_params.yaml')],
        ),
        Node(
            package='pasqua_tf',
            namespace='tf_tree',
            executable='tf_tree_rear_camera',
            name='tf_tree_rear_camera',
            output='screen',
        ),
])