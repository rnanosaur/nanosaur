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


from genericpath import isfile
import os
import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_control = launch_ros.substitutions.FindPackageShare(package='nanosaur_control').find('nanosaur_control')
    
    default_config_locks = os.path.join(get_package_share_directory('twist_mux'),
                                        'config', 'twist_mux_locks.yaml')
    default_config_topics = os.path.join(get_package_share_directory('twist_mux'),
                                         'config', 'twist_mux_topics.yaml')
    default_config_joystick = os.path.join(get_package_share_directory('twist_mux'),
                                           'config', 'joystick.yaml')
    
    # https://answers.ros.org/question/306935/ros2-include-a-launch-file-from-a-launch-file/
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [pkg_control, '/launch/teleop.launch.py']),
            launch_arguments = {'joy_vel': 'joy_vel',
                                'config_filepath': os.path.join(pkg_control, 'param', 'ps3.nanosaur.yml')}.items(),
        )

    twist_mux_node = launch_ros.actions.Node(
            package='twist_mux',
            executable='twist_mux',
            output='screen',
            remappings={('/cmd_vel_out', LaunchConfiguration('cmd_vel_out'))},
            parameters=[
                LaunchConfiguration('config_locks'),
                LaunchConfiguration('config_topics'),
                LaunchConfiguration('config_joy')]
        )

    return launch.LaunchDescription([
        DeclareLaunchArgument(
            'config_locks',
            default_value=default_config_locks,
            description='Default locks config file'),
        DeclareLaunchArgument(
            'config_topics',
            default_value=default_config_topics,
            description='Default topics config file'),
        DeclareLaunchArgument(
            'config_joy',
            default_value=default_config_joystick,
            description='Default joystick config file'),
        DeclareLaunchArgument(
            'cmd_vel_out',
            default_value='cmd_vel',
            description='cmd vel output topic'),
        # teleoperation joystick nanosaur
        teleop_launch,
        # Twist mux launcher
        twist_mux_node,
    ])
# EOF