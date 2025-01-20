# Copyright (C) 2024, Raffaello Bonghi <raffaello@rnext.it>
# All rights reserved
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright 
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its 
#    contributors may be used to endorse or promote products derived 
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
# CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, 
# BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    
    nanosaur_description_path = get_package_share_directory('nanosaur_description')
    
    gui = LaunchConfiguration('gui')
    camera_type = LaunchConfiguration('camera_type')
    lidar_type = LaunchConfiguration('lidar_type')
    rvizconfig = LaunchConfiguration('rvizconfig')
    
    default_rviz_config_path = os.path.join(nanosaur_description_path, 'rviz', 'urdf.rviz')

    declare_camera_type_cmd = DeclareLaunchArgument(
        name='camera_type',
        default_value='empty',
        description='camera type to use. Options: empty, Realsense, zed.')

    declare_lidar_type_cmd = DeclareLaunchArgument(
        name='lidar_type',
        default_value='empty',
        description='Lidar type to use. Options: empty, LD06.')

    declare_gui_cmd = DeclareLaunchArgument(
        name='gui',
        default_value='True',
        description='Flag to enable joint_state_publisher_gui')

    declare_rvizconfig_cmd = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=default_rviz_config_path,
        description='Absolute path to rviz config file')

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(gui),
        on_exit=Shutdown()
    )
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(gui)
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rvizconfig],
        on_exit=Shutdown()
    )

    # Nanosaur description launch
    # https://answers.ros.org/question/306935/ros2-include-a-launch-file-from-a-launch-file/
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nanosaur_description_path, '/launch/description.launch.py']),
        launch_arguments={'camera_type': camera_type, 'lidar_type': lidar_type}.items()
        )

    # Define LaunchDescription variable and return it
    ld = LaunchDescription()
    
    ld.add_action(declare_camera_type_cmd)
    ld.add_action(declare_lidar_type_cmd)
    ld.add_action(declare_gui_cmd)
    ld.add_action(declare_rvizconfig_cmd)
    ld.add_action(description_launch)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(joint_state_publisher_gui_node)
    ld.add_action(rviz_node)

    return ld
# EOF
