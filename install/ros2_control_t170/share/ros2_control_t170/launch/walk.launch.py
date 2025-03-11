# Copyright 2023 ros2_control Development Team
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

from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
   # Load URDF
    urdf_path = os.path.join(get_package_share_directory('ros2_control_t170'), 'urdf', 'ti5.urdf')
    urdf = open(urdf_path).read()
    robot_description = {"robot_description": urdf}
    walk_node = Node(
        package="ros2_control_t170",
        executable="walk",
        name="walk_node",
        parameters=[robot_description],
    )

    nodes_to_start = [walk_node]
    return LaunchDescription(nodes_to_start)
