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


import rclpy
import math
from urdf_parser_py.urdf import URDF
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from .motors import Motors


def euclidean_of_vectors(xyz1, xyz2):
    return math.sqrt(
        (xyz1[0] - xyz2[0]) ** 2 +
        (xyz1[1] - xyz2[1]) ** 2 +
        (xyz1[2] - xyz2[2]) ** 2)


def get_joint(robot, joint_name):
    for joint in robot.joints:
        if joint.name == joint_name:
            return joint
    raise Exception(f"Joint {joint_name} not found")


def get_link(robot, link_name):
    for link in robot.links:
        if link.name == link_name:
            return link
    raise Exception(f"link {link_name} not found")


class NanoSaur(Node):

    def __init__(self):
        super().__init__('nanosaur')
        # Load motors
        self.motors = Motors()
        timer_period = 1  # seconds
        qos_profile = QoSProfile(depth=10)
        self.create_subscription(
            String, 'robot_description',
            lambda msg: self.configure_robot(msg.data),
            rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL))
        # joint state controller
        # https://index.ros.org/doc/ros2/Tutorials/URDF/Using-URDF-with-Robot-State-Publisher/
        self.joint_state = JointState()
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.timer = self.create_timer(timer_period, self.transform_callback)
        # Drive control
        self.v = 0.0
        self.w = 0.0
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.drive_callback,
            10)
        self.subscription  # prevent unused variable warning
        # Node started
        self.get_logger().info("Hello NanoSaur!")

    def configure_robot(self, description):
        self.get_logger().info('Got description, configuring robot')
        # Load description
        # From https://github.com/ros-controls/ros_controllers/blob/noetic-devel/diff_drive_controller/src/diff_drive_controller.cpp
        robot = URDF.from_xml_string(description)

        # Get parameter left wheel name
        # https://index.ros.org/doc/ros2/Tutorials/Using-Parameters-In-A-Class-Python/
        self.declare_parameter("left_wheel")
        left_wheel_name = self.get_parameter("left_wheel").get_parameter_value().string_value
        self.get_logger().debug(f"Right wheel name: {left_wheel_name}")
        # Get left joint wheel
        joint_left = get_joint(robot, left_wheel_name)

        # Get parameter right wheel name
        self.declare_parameter("right_wheel")
        right_wheel_name = self.get_parameter("right_wheel").get_parameter_value().string_value
        self.get_logger().debug(f"Left wheel name: {right_wheel_name}")
        # Get right joint wheel
        joint_right = get_joint(robot, right_wheel_name)
        # Measure distance
        self.distance = euclidean_of_vectors(joint_left.origin.xyz, joint_right.origin.xyz)
        # Get radius joint
        link_left = get_link(robot, joint_left.child)
        self.radius = link_left.collision.geometry.radius

        self.get_logger().info(f"Distance {self.distance} - Radius {self.radius}")

    def drive_callback(self, msg):
        # Store linear velocity and angular velocity
        self.v = msg.linear.x
        self.w = msg.angular.z
        self.get_logger().info(f"v={self.v} w={self.w}")

    def transform_callback(self):
        now = self.get_clock().now()
        self.get_logger().info(f"TF update callback")
        # send the joint state and transform
        self.joint_state.header.stamp = now.to_msg()
        self.joint_state.name = ['sprocket_left_joint', 'sprocket_right_joint']
        self.joint_state.position = [0.5, 0.5]
        self.joint_state.velocity = [0.1, 0.1]
        self.joint_pub.publish(self.joint_state)


def main(args=None):
    rclpy.init(args=args)

    nanosaur = NanoSaur()
    try:
        rclpy.spin(nanosaur)
    except KeyboardInterrupt:
        pass
    # Destroy the node explicitly
    nanosaur.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
# EOF
