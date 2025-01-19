# Copyright (C) 2025, Raffaello Bonghi <raffaello@rnext.it>
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
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    
    nanosaur_visualization = get_package_share_directory('nanosaur_visualization')
    
    head_type = LaunchConfiguration('head_type')
    flap_type = LaunchConfiguration('flap_type')
    namespace = LaunchConfiguration('namespace', default="nanosaur")
    rvizconfig = LaunchConfiguration('rvizconfig')
    
    default_rviz_config_path = os.path.join(nanosaur_visualization, 'rviz', 'robot.rviz')

    nanosaur_cmd = DeclareLaunchArgument(
        name='namespace',
        default_value='nanosaur',
        description='nanosaur namespace name. If you are working with multiple robot you can change this namespace.')

    declare_head_type_cmd = DeclareLaunchArgument(
        name='head_type',
        default_value='realsense',
        description='Head type to use. Options: empty, Realsense, zed.')

    declare_flap_type_cmd = DeclareLaunchArgument(
        name='flap_type',
        default_value='empty',
        description='Flap type to use. Options: empty, LD06.')
    
    declare_rvizconfig_cmd = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=default_rviz_config_path,
        description='Absolute path to rviz config file')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        namespace=namespace,
        arguments=['-d', rvizconfig],
    )

    # Define LaunchDescription variable and return it
    ld = LaunchDescription()
    ld.add_action(nanosaur_cmd)
    ld.add_action(declare_head_type_cmd)
    ld.add_action(declare_flap_type_cmd)
    ld.add_action(declare_rvizconfig_cmd)
    ld.add_action(rviz_node)

    return ld
# EOF
