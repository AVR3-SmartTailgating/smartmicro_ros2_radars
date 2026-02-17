# Copyright (c) 2021, s.m.s, smart microwave sensors GmbH, Brunswick, Germany
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
import launch_ros

from ament_index_python import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription

PACKAGE_NAME = 'umrr_ros2_driver'


def generate_launch_description():
    """Generate the launch description.

    Note: CAN interface should be configured using the setup script:
        sudo ./scripts/setup_can_interface.sh can0 500000

    This installs udev rules for automatic CAN interface configuration.
    """

    radar__params = os.path.join(
           get_package_share_directory(PACKAGE_NAME), 'param/radar.params.yaml')

    filter__params = os.path.join(
           get_package_share_directory(PACKAGE_NAME), 'param/radar_filter.params.yaml')

    # RViz2 configuration file
    rviz_config = os.path.join(
        get_package_share_directory('smart_rviz_plugin'),
        'config/rviz/drv152.rviz'
    )

    radar_node = Node(
        package=PACKAGE_NAME,
        executable='smartmicro_radar_node_exe',
        name='smart_radar',
        parameters=[radar__params],
        respawn=False,
        #respawn_delay=2.0,
        output='screen'
    )

    filter_node = Node(
        package=PACKAGE_NAME,
        executable='radar_filter_node_exe',
        name='radar_filter',
        parameters=[filter__params],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        radar_node,
        filter_node,
        rviz_node
    ])

    
