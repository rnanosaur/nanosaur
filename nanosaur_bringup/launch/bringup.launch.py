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


import os
import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros


def generate_launch_description():
    pkg_bringup = launch_ros.substitutions.FindPackageShare(package='nanosaur_bringup').find('nanosaur_bringup')
    pkg_description = launch_ros.substitutions.FindPackageShare(package='nanosaur_description').find('nanosaur_description')
    pkg_camera = launch_ros.substitutions.FindPackageShare(package='nanosaur_camera').find('nanosaur_camera')

    nanosaur_dir = LaunchConfiguration(
        'nanosaur_dir',
        default=os.path.join(pkg_bringup, 'param', 'nanosaur.yml'))

    nanosaur_node = launch_ros.actions.Node(
        package='nanosaur_robot',
        executable='nanosaur',
        name='nanosaur',
        parameters=[nanosaur_dir],
        output='screen'
    )

    return launch.LaunchDescription([
        DeclareLaunchArgument(
            'nanosaur_dir',
            default_value=nanosaur_dir,
            description='Full path to nanosaur parameter file to load'),

        # https://answers.ros.org/question/306935/ros2-include-a-launch-file-from-a-launch-file/
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [pkg_description, '/launch/description.launch.py'])),
        # Camera launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [pkg_camera, '/launch/camera.launch.py'])),
        # Launch nanusaur driver
        nanosaur_node
    ])
# EOF