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
import time

from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class CycleLifecycleNode(Node):
    def __init__(self):
        super().__init__("cycle_lifecycle")
        self.get_client = self.create_client(GetState, "aic_model/get_state")
        self.change_client = self.create_client(ChangeState, "aic_model/change_state")
        while not self.get_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("waiting for get_state service...")
        while not self.change_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("waiting for change_state service...")

    def get_model_state(self):
        future = self.get_client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        return future.result()

    def change_model_state(self, transition_id):
        request = ChangeState.Request()
        request.transition.id = transition_id
        future = self.change_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        result = future.result()
        if result.success:
            self.get_logger().info(f"transitioned to {transition_id} successfully")
        else:
            self.get_logger().error(f"unable to transition to {transition_id}")
        return result.success


def main(args=None):
    with rclpy.init(args=args):
        node = CycleLifecycleNode()
        state = node.get_model_state()
        print(f"current state: {state}")

        print(f"configuring...")
        node.change_model_state(Transition.TRANSITION_CONFIGURE)
        time.sleep(1.0)

        print(f"activating...")
        node.change_model_state(Transition.TRANSITION_ACTIVATE)
        time.sleep(1.0)

        print(f"deactivating...")
        node.change_model_state(Transition.TRANSITION_DEACTIVATE)
        time.sleep(1.0)

        print(f"shutting down...")
        node.change_model_state(Transition.TRANSITION_INACTIVE_SHUTDOWN)


if __name__ == "__main__":
    main()
