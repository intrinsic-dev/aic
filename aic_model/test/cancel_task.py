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

import rclpy

from aic_task_interfaces.action import InsertCable
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import Empty


class CancelAllTasksNode(Node):
    def __init__(self):
        super().__init__("test_cancel_task")
        self.client = self.create_client(Empty, "cancel_task")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("waiting for cancel_task service...")

    def call_cancel_task(self):
        future = self.client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)


def main(args=None):
    with rclpy.init(args=args):
        node = CancelAllTasksNode()
        node.call_cancel_task()


if __name__ == "__main__":
    main()
