#!/usr/bin/env python3

#
#  Copyright (C) 2025 Intrinsic Innovation LLC
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
#

import sys
import time
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import WrenchStamped

class ControlReferencePublisher(Node):
    def __init__(self):
        super().__init__('control_reference_publisher')

        self.declare_parameter('controller_namespace', '')
        self.declare_parameter('tool_frame', '')
        self.declare_parameter('contact_force_z', 0.0)
        self.controller_namespace = self.get_parameter('controller_namespace').get_parameter_value().string_value
        self.tool_frame = self.get_parameter('tool_frame').get_parameter_value().string_value
        self.contact_force_z = self.get_parameter('contact_force_z').get_parameter_value().double_value

        self.joint_ref_publisher = self.create_publisher(
            JointTrajectoryPoint, self.controller_namespace+'/joint_references', 10)
        self.wrench_ref_publisher = self.create_publisher(
            WrenchStamped, self.controller_namespace+'/wrench_reference', 10)
        
        self.timer = self.create_timer(1.0, self.send_references)

    def generate_joint_ref(self, positions = [], velocities = [], accelerations = [], effort = []):
        msg = JointTrajectoryPoint()
        msg.positions = positions
        msg.velocities = velocities
        msg.accelerations = accelerations
        msg.effort = effort
        return msg

    def generate_wrench_stamped(self, f_x, f_y, f_z):
        msg = WrenchStamped()
        msg.header.frame_id = self.tool_frame
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.wrench.force.x = f_x
        msg.wrench.force.y = f_y
        msg.wrench.force.z = f_z

        msg.wrench.torque.x = 0.0
        msg.wrench.torque.y = 0.0
        msg.wrench.torque.z = 0.0

        return msg

    def send_references(self):
        print("Sending wrench references")
        # Exert force
        wrench_ref = self.generate_wrench_stamped(0.0, 0.0, self.contact_force_z)
        self.wrench_ref_publisher.publish(wrench_ref)

def main(args=None):
    rclpy.init(args=args)

    control_ref_node = ControlReferencePublisher()

    control_ref_node.send_references()

    rclpy.spin(control_ref_node)

    control_ref_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main(sys.argv)


