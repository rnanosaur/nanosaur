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
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def launch_setup(context: LaunchContext, support_package):
    """ Reference:
        https://answers.ros.org/question/396345/ros2-launch-file-how-to-convert-launchargument-to-string/ 
        https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/main/ur_moveit_config/launch/ur_moveit.launch.py
    """
    # render namespace, dumping the support_package.
    namespace = context.perform_substitution(support_package)

    xacro_path = LaunchConfiguration('xacro_path')
    cover_type = LaunchConfiguration('cover_type')
    diff_drive_emulation = LaunchConfiguration('diff_drive_emulation')
    use_nominal_extrinsics = LaunchConfiguration('use_nominal_extrinsics')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            #'frame_prefix': f"{namespace}/", # Reimplemented https://github.com/ros/robot_state_publisher/pull/169
            'robot_description': Command(
                [
                    'xacro ', xacro_path, ' ',
                    'cover_type:=', cover_type, ' ',
                    'diff_drive_emulation:=', diff_drive_emulation, ' ',
                    'use_nominal_extrinsics:=', use_nominal_extrinsics, ' ',
                ])
        }]
    )
    return [robot_state_publisher_node]


def generate_launch_description():

    namespace = LaunchConfiguration('namespace', default="nanosaur")

    # URDF/xacro file to be loaded by the Robot State Publisher node
    default_xacro_path = os.path.join(
        get_package_share_directory('nanosaur_description'),
        'urdf',
        'nanosaur.urdf.xml'
    )

    declare_model_path_cmd = DeclareLaunchArgument(
        name='xacro_path',
        default_value=default_xacro_path,
        description='Absolute path to robot urdf file')
    
    nanosaur_cmd = DeclareLaunchArgument(
        name='namespace',
        default_value='nanosaur',
        description='nanosaur namespace name. If you are working with multiple robot you can change this namespace.')

    declare_cover_type_cmd = DeclareLaunchArgument(
        name='cover_type',
        default_value='fisheye',
        description='Cover type to use. Options: pi, fisheye, realsense, zed.')

    declare_simulation_cmd = DeclareLaunchArgument(
        name='diff_drive_emulation',
        default_value='false',
        description='Enable urdf for differential drive emulation, for simulation.')

    declare_use_nominal_extrinsics_cmd = DeclareLaunchArgument(
        name='use_nominal_extrinsics',
        default_value='false',
        description='Use nominal extrinsics ONLY for Realsense camera.')

    # Define LaunchDescription variable and return it
    ld = LaunchDescription()    
    ld.add_action(nanosaur_cmd)
    ld.add_action(declare_model_path_cmd)
    ld.add_action(declare_simulation_cmd)
    ld.add_action(declare_cover_type_cmd)
    ld.add_action(declare_use_nominal_extrinsics_cmd)
    ld.add_action(OpaqueFunction(function=launch_setup, args=[namespace]))

    return ld
# EOF
