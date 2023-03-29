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
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from launch import LaunchDescription


def generate_launch_description():
    ld = LaunchDescription()
    slave_eds_path = os.path.join(
                    get_package_share_directory("trinamic_pd42_can"), "config", "single-pd42", "TMCM-1270.eds"
                )

    slave_node_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("canopen_fake_slaves"), "launch"
                ),
                "/cia402_slave.launch.py",
            ]
        ),
        launch_arguments={
            "node_id": "1", 
            "node_name": "pd42_slave",
            "slave_config": slave_eds_path,
            }.items(),
    )
    master_bin_path = os.path.join(
        get_package_share_directory("trinamic_pd42_can"), 
        "config",
        "single-pd42",
        "master.bin",
    )
    if not os.path.exists(master_bin_path):
        master_bin_path = ""

    device_container = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("canopen_core"), "launch"),
                "/canopen.launch.py",
            ]
        ),
        launch_arguments={
            "master_config": os.path.join(
                get_package_share_directory("trinamic_pd42_can"),
                "config",
                "single-pd42",
                "master.dcf",
            ),
            "master_bin": master_bin_path,
            "bus_config": os.path.join(
                get_package_share_directory("trinamic_pd42_can"),
                "config",
                "single-pd42",
                "bus.yml",
            ),
            "can_interface_name": "vcan0",
        }.items(),
    )

    # ld.add_action(device_container)
    ld.add_action(slave_node_1)

    return ld
