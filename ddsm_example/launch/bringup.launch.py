# Copyright 2026 Dosymzhan Birzhanuly
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

"""
Drive a DDSM115 robot with ros2_control.

    ros2 launch ddsm_example bringup.launch.py port:=/dev/ttyUSB0 wheels:=4
    ros2 run teleop_twist_keyboard teleop_twist_keyboard     # publishes /cmd_vel

Starts robot_state_publisher, ros2_control_node with the DDSM115 hardware plugin, and the
joint_state_broadcaster and diff_drive_controller. Commands come from /cmd_vel, wheel odometry
goes to /odom (and TF odom -> base_link).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

# Defaults per wheel count; the order is the joint order of urdf/robot.urdf.xacro
DEFAULTS = {
    '4': {'motor_ids': '1,2,3,4', 'motor_directions': '1,1,-1,-1',
          'motor_trims': '1.0,1.0,1.0,1.0'},
    '2': {'motor_ids': '1,2', 'motor_directions': '1,-1', 'motor_trims': '1.0,1.0'},
}


def launch_setup(context):
    share = get_package_share_directory('ddsm_example')
    wheels = LaunchConfiguration('wheels').perform(context)
    if wheels not in DEFAULTS:
        raise RuntimeError('wheels must be 2 or 4')

    mappings = {'wheels': wheels, 'port': LaunchConfiguration('port').perform(context)}
    for name, default in DEFAULTS[wheels].items():
        mappings[name] = LaunchConfiguration(name).perform(context) or default
    robot_description = xacro.process_file(
        os.path.join(share, 'urdf', 'robot.urdf.xacro'), mappings=mappings).toxml()

    controllers = os.path.join(share, 'config', f'controllers_{wheels}wd.yaml')
    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[controllers],
            output='both',
            remappings=[
                ('/controller_manager/robot_description', '/robot_description'),
                ('/diff_cont/cmd_vel_unstamped', '/cmd_vel'),
                ('/diff_cont/odom', '/odom'),
            ],
        ),
        *[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[name, '--controller-manager-timeout', '30'],
            )
            for name in ('joint_broad', 'diff_cont')
        ],
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0',
                              description='USB-RS485 adapter, see ls -l /dev/serial/by-id/'),
        DeclareLaunchArgument('wheels', default_value='4', description='2 or 4'),
        DeclareLaunchArgument('motor_ids', default_value='',
                              description='comma separated, joint order (default 1,2,3,4)'),
        DeclareLaunchArgument('motor_directions', default_value='',
                              description='1 or -1 per motor (default: right side -1)'),
        DeclareLaunchArgument('motor_trims', default_value='',
                              description='speed factor per motor (default 1.0)'),
        OpaqueFunction(function=launch_setup),
    ])
