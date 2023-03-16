# Copyright 2023 Amadeusz Szymko
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
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import PushRosNamespace


def launch_setup(context, *args, **kwargs):
    sensor_launch_pkg = LaunchConfiguration("sensor_model").perform(context) + "_launch"
    
    sensing_launch = GroupAction([
        PushRosNamespace("sensing"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                launch_file_path=PathJoinSubstitution([
                    FindPackageShare(sensor_launch_pkg), "launch", "sensing.launch.py"
                ]),
            ),
            launch_arguments={
                "container_name": LaunchConfiguration("container_name"),
                "use_laser_container": LaunchConfiguration("use_laser_container"),
                "use_multithread": LaunchConfiguration("use_multithread"),
                "use_intra_process": LaunchConfiguration("use_intra_process"),
            }.items()
        )
    ])

    return [
        sensing_launch
    ]


def generate_launch_description():
    declared_arguments = []
    
    def add_launch_arg(name: str, default_value=None):
        declared_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value)
        )
    
    add_launch_arg("sensor_model")
    add_launch_arg("container_name", "hokuyo_node_container")
    add_launch_arg("use_laser_container", "false")
    add_launch_arg("use_multithread", "false")
    add_launch_arg("use_intra_process", "false")

    return LaunchDescription([
        *declared_arguments,
        OpaqueFunction(function=launch_setup)
    ])
