# Copyright (C) 2021, Raffaello Bonghi <raffaello@rnext.it>
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

# This launch file is a modified version from
# Check PR https://github.com/ros2/teleop_twist_joy/pull/29
# https://github.com/ros2/teleop_twist_joy

import os

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    joy_config = LaunchConfiguration('joy_config')
    joy_dev = LaunchConfiguration('joy_dev')
    config_filepath = LaunchConfiguration('config_filepath')

    joy_ns_cmd = DeclareLaunchArgument(
        'joy_ns',
        default_value='teleop',
        description='Enable a namespace for multiple robot. This namespace group all nodes and topics.')

    joy_vel_cmd = DeclareLaunchArgument('joy_vel', default_value='cmd_vel')

    config_filepath_cmd = DeclareLaunchArgument(
        'config_filepath', default_value='ps3')

    joy_dev_cmd = DeclareLaunchArgument(
        'joy_dev', default_value='/dev/input/js0')
    
    joy_ns = LaunchConfiguration('joy_ns')
    joy_vel = LaunchConfiguration('joy_vel')

    #config_filepath_cmd = DeclareLaunchArgument('config_filepath', default_value=[
    #    launch.substitutions.TextSubstitution(text=os.path.join(
    #        get_package_share_directory('teleop_twist_joy'), 'config', '')),
    #    joy_config, launch.substitutions.TextSubstitution(text='.config.yaml')]),

    joy_node = Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            namespace=joy_ns,
            parameters=[{
                'dev': joy_dev,
                'deadzone': 0.3,
                'autorepeat_rate': 20.0,
            }])
    
    teleop_twist_node = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            namespace=joy_ns,
            parameters=[config_filepath],
            remappings=[('cmd_vel', joy_vel)],
        )

    ld = LaunchDescription()
    
    ld.add_action(joy_ns_cmd)
    ld.add_action(joy_vel_cmd)
    ld.add_action(config_filepath_cmd)
    ld.add_action(joy_dev_cmd)
    ld.add_action(joy_node)
    ld.add_action(teleop_twist_node)

    return ld
# EOF
