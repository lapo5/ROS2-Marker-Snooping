from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
import sys
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

from launch.actions.execute_process import ExecuteProcess

def generate_launch_description():

    params_ptu = os.path.join(get_package_share_directory("hal_flir_d46"), 'params', 'params_pasqualone.yaml')
    params_marker_snooping = os.path.join(get_package_share_directory("marker_snooping"), 'params', 'params_pasqualone.yaml')
    params_aruco_filter = os.path.join(get_package_share_directory("aruco_pose_filter"), 'params', 'params_pasqualone.yaml')
    
    for arg in sys.argv:
        if arg.startswith("project:="):
            project = arg.split(":=")[1]
            params_ptu = os.path.join(get_package_share_directory("hal_flir_d46"), 'params', 'params_' + project + '.yaml')
            params_marker_snooping = os.path.join(get_package_share_directory("marker_snooping"), 'params', 'params_' + project + '.yaml')
            params_aruco_filter = os.path.join(get_package_share_directory("aruco_pose_filter"), 'params', 'params_' + project + '.yaml')
    
    return LaunchDescription([
        Node(
            package='simulation_utilities',
            executable='simulate_marker',
            name='simulate_marker',
            output={
                    "stdout": "screen",
                    "stderr": "screen",
            },
        ),
        Node(
            package='hal_flir_d46',
            executable='hal_flir_d46',
            name='hal_flir_d46',
            output={
                    "stdout": "screen",
                    "stderr": "screen",
            },
            parameters=[params_ptu],
        ),
        Node(
            package='aruco_pose_filter',
            executable='pose_filter',
            name='pose_filter',
            output={
                    "stdout": "screen",
                    "stderr": "screen",
            },
            parameters=[params_aruco_filter, {'marker_id': '69'}],
        ),
        Node(
            package='marker_snooping',
            executable='marker_snooping',
            name='marker_snooping',
            output={
                    "stdout": "screen",
                    "stderr": "screen",
            },
            parameters=[params_marker_snooping],
        ),
])
