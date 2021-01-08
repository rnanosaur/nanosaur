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
from .motor import Motor


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
        # Get rate joint_states
        self.declare_parameter("rate", 5)
        self.timer_period = 1. / float(self.get_parameter("rate").value)
        self.get_logger().debug(f"timer {self.timer_period}")
        # Get RPM motors
        self.declare_parameter("rpm")
        self.rpm = self.get_parameter("rpm").value
        self.get_logger().debug(f"RPM motors {self.rpm}")
        # Get parameter left wheel name
        # https://index.ros.org/doc/ros2/Tutorials/Using-Parameters-In-A-Class-Python/
        self.declare_parameter("left_wheel")
        self.left_wheel_name = self.get_parameter("left_wheel").value
        self.get_logger().debug(f"Right wheel name: {self.left_wheel_name}")
        # Get parameter right wheel name
        self.declare_parameter("right_wheel")
        self.right_wheel_name = self.get_parameter("right_wheel").value
        self.get_logger().debug(f"Left wheel name: {self.right_wheel_name}")
        # Load motors
        left_id = 1
        right_id = 2
        self.mright = Motor(right_id, self.rpm)
        self.mleft = Motor(left_id, self.rpm)
        # Load subscriber robot_description
        self.create_subscription(
            String, 'robot_description',
            lambda msg: self.configure_robot(msg.data),
            QoSProfile(depth=1, durability=rclpy.qos.QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL))
        # joint state controller
        # https://index.ros.org/doc/ros2/Tutorials/URDF/Using-URDF-with-Robot-State-Publisher/
        self.joint_state = JointState()
        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.timer = self.create_timer(self.timer_period, self.transform_callback)
        # Drive control
        self.p = [0.0, 0.0] # [right, left]
        self.r = [0.0, 0.0] # [right, left]
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
        # Get left joint wheel
        joint_left = get_joint(robot, self.left_wheel_name)
        # Get right joint wheel
        joint_right = get_joint(robot, self.right_wheel_name)
        # Measure wheel separation
        self.wheel_separation = euclidean_of_vectors(joint_left.origin.xyz, joint_right.origin.xyz)
        # Get radius joint
        link_left = get_link(robot, joint_left.child)
        self.radius = link_left.collision.geometry.radius
        self.get_logger().info(f"Wheel separation {self.wheel_separation} - Radius {self.radius}")

    def convert_speed(self, v, w):
        half_wheel_separation = self.wheel_separation / 2.
        vr = v + half_wheel_separation * w
        vl = v - half_wheel_separation * w
        rr = vr / self.radius
        rl = vl / self.radius
        return [rr, rl]

    def drive_callback(self, msg):
        # Store linear velocity and angular velocity
        v = msg.linear.x
        w = msg.angular.z
        # Convert linear and angular velocity to radiant motor speed
        self.get_logger().debug(f"v={v} w={w}")
        r = self.convert_speed(v, w)
        self.get_logger().debug(f"rad {r}")
        max_speed = self.rpm / 60.
        # Constrain between -max_speed << speed << max_speed.
        self.r = [max(-max_speed, min(max_speed, r[0])), max(-max_speed, min(max_speed, r[1]))]
        # Send a warning message if speed is over 
        if r[0] != self.r[0]:
            self.get_logger().warning(f"ref speed over {r[0] - self.r[0]}")
        if r[1] != self.r[1]:
            self.get_logger().warning(f"ref speed over {r[1] - self.r[1]}")
        # Convert speed to motor speed in RPM
        rpmr = self.r[0] * 60.
        rpml = self.r[1] * 60.
        # Set to max RPM available
        self.get_logger().info(f"RPM R={rpmr} L={rpml}")
        self.mright.set_speed(rpmr)
        self.mleft.set_speed(rpml)

    def transform_callback(self):
        now = self.get_clock().now()
        # send the joint state and transform
        self.joint_state.header.stamp = now.to_msg()
        self.joint_state.name = [self.left_wheel_name, self.right_wheel_name]
        # Estimate angle position
        self.p[0] = (self.p[0] + self.r[0] * self.timer_period) % (2 * math.pi)
        self.p[1] = (self.p[1] + self.r[1] * self.timer_period) % (2 * math.pi)
        self.joint_state.position = self.p
        self.joint_state.velocity = self.r
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
