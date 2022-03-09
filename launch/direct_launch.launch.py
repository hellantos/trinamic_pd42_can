# Copyright 2022 Christoph Hellmann Santos
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))  # noqa
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'launch'))  # noqa

import launch
import launch.actions
import launch.events

import launch_ros
import launch_ros.events  
import launch_ros.events.lifecycle

import lifecycle_msgs.msg


def generate_launch_description():
    path_to_test = os.path.dirname(__file__)

    ld = launch.LaunchDescription()

    master_node = launch_ros.actions.Node(
        name="device_manager_node",
        namespace="", 
        package="ros2_canopen_core", 
        output="screen", 
        executable="device_manager_node",
        parameters= [{
            "bus_config": os.path.join(path_to_test, ".." ,  "config" , "bus.yml"),
            "master_config": os.path.join(path_to_test, ".." , "config" , "master.dcf"),
            "master_bin": os.path.join(path_to_test, ".." , "config" , "master.bin"),
            "can_interface_name": "can0",
            "enable_lazy_loading": False
            }
        ],
    )

    ld.add_action(master_node)

    return ld
