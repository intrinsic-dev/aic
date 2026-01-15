#
#  Copyright (C) 2026 Intrinsic Innovation LLC
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


import numpy as np


class AicModelPolicy:
    def __init__(self, parent_node):
        self._parent_node = parent_node
        print("AicModel.__init__()")
        # YOUR CODE HERE

    def start_callback(self, task):
        print("start_callback()")
        # YOUR CODE HERE

    def stop_callback(self):
        print("stop_callback()")
        # YOUR CODE HERE

    def observation_callback(self, observation):
        print("observation_callback()")
        # YOUR CODE HERE
        # self._parent_node.set_pose_target(something)
