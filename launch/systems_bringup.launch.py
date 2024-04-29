# Copyright 2024 Intelligent Robotics Lab
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

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('systems_bringup')
    robocup_dir = get_package_share_directory('robocup_bringup')
    nav_dir = get_package_share_directory('navigation_system')
    yolo_dir = get_package_share_directory('yolov8_bringup')

    namespace = LaunchConfiguration('namespace')
    rviz = LaunchConfiguration('rviz')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')
    
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz',
        default_value='False',
        description='Open RViz')

    navigation_system_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav_dir, 'launch', 'navigation_system.launch.py')),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': 'false',
            'rviz': rviz,
            'map': os.path.join(robocup_dir, 'maps', 'lab_robocup.yaml'),
            'params_file': os.path.join(robocup_dir, 'params', 'tiago_nav_params.yaml')
        }.items()
    )

    config = os.path.join(os.path.join(pkg_dir, 'config', 'params.yaml'))

    system_manager_cmd = Node(
        package='systems_bringup',
        executable='systems_main',
        output='screen',
        namespace=namespace,
        parameters=[config])

    yolo3d_launch = os.path.join(yolo_dir, 'launch', 'yolov8_3d.launch.py')
    yolo3d = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(yolo3d_launch),
        launch_arguments={
            'namespace': namespace,
            'model': 'yolov8m.pt',
            'input_image_topic': '/head_front_camera/rgb/image_raw',
            'input_depth_topic': '/head_front_camera/depth_registered/image_raw',
            'input_depth_info_topic': '/head_front_camera/depth_registered/camera_info',
            'depth_image_units_divisor': '1',  # 1 for simulation, 1000 in real robot
            'target_frame': 'head_front_camera_link',
            'threshold': '0.5'
            }.items()
    )

    dialog_system_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(robocup_dir, 'launch', 'dialog.launch.py'))
    )

    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(navigation_system_cmd)
    ld.add_action(dialog_system_cmd)
    ld.add_action(system_manager_cmd)
    ld.add_action(yolo3d)

    return ld
