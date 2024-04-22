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
    yolo_dir = get_package_share_directory('yolov8_bringup')

    namespace = LaunchConfiguration('namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')

    config = os.path.join(os.path.join(pkg_dir, 'config', 'params.yaml'))

    attention_manager_cmd = Node(
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

    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)
    ld.add_action(attention_manager_cmd)
    ld.add_action(yolo3d)

    return ld
