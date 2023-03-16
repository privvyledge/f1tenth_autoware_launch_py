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
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def launch_setup(context, *args, **kwargs):
    pkg_prefix = FindPackageShare("f1tenth_launch")
    config_rviz = PathJoinSubstitution([pkg_prefix, "rviz", LaunchConfiguration("rviz_config")])
    map_yaml = PathJoinSubstitution(
        [LaunchConfiguration("map_path"),
         LaunchConfiguration("map_yaml")])

    global_parameter_loader_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([
                FindPackageShare("global_parameter_loader"), "launch", "global_params.launch.py"
            ]),
        ),
        launch_arguments={
            "use_sim_time": "true",
            "vehicle_model": "f1tenth_vehicle",
        }.items()
    )

    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution(
                [FindPackageShare("f1tenth_launch"),
                 "launch/components", "f1tenth_control.launch.py"]),),
        launch_arguments={}.items(),
        condition=IfCondition(LaunchConfiguration("launch_control")))

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution(
                [FindPackageShare("f1tenth_launch"),
                 "launch/components", "f1tenth_localization.launch.py"]),),
        launch_arguments={"map_yaml": map_yaml, }.items(),
        condition=IfCondition(LaunchConfiguration("launch_localization")))

    map_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([
                FindPackageShare("f1tenth_launch"), "launch/components", "f1tenth_map.launch.py"
            ]),
        ),
        launch_arguments={
            "map": LaunchConfiguration("map_path"),
            "use_intra_process": "false",
            "use_multithread": "false",
        }.items(),
        condition=IfCondition(LaunchConfiguration("launch_map"))
    )

    planning_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution(
                [FindPackageShare("f1tenth_launch"),
                 "launch/components", "f1tenth_planning.launch.py"]),),
        launch_arguments={}.items(),
        condition=IfCondition(LaunchConfiguration("launch_planning")))

    sensing_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution(
                [FindPackageShare("f1tenth_launch"),
                 "launch/components", "f1tenth_sensing.launch.py"]),),
        launch_arguments={"sensor_model": LaunchConfiguration("sensor_model"),
                          "container_name": "hokuyo_node_container",
                          "use_laser_container": "false", "use_multithread": "false",
                          "use_intra_process": "false", }.items(),
        condition=IfCondition(LaunchConfiguration("launch_sensing")))

    system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([
                FindPackageShare("f1tenth_launch"), "launch/components", "f1tenth_system.launch.py"
            ]),
        ),
        launch_arguments={
            "system_run_mode": LaunchConfiguration("system_run_mode"),
            "launch_system_monitor": LaunchConfiguration("launch_system_monitor"),
            "launch_dummy_diag_publisher": LaunchConfiguration("launch_dummy_diag_publisher"),
            "sensor_model": LaunchConfiguration("sensor_model"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("launch_system"))
    )

    vehicle_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([
                FindPackageShare("f1tenth_launch"), "launch/components", "f1tenth_vehicle.launch.py"
            ]),
        ),
        launch_arguments={
            "vehicle_model": "f1tenth_vehicle",
            "sensor_model": "f1tenth_sensor_kit",
            "launch_vehicle_interface": "true",
        }.items(),
        condition=IfCondition(LaunchConfiguration("launch_vehicle"))
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", str(config_rviz.perform(context)), "-s",
                   str(PathJoinSubstitution([FindPackageShare(
                       "autoware_launch"), "rviz/image/autoware.png"])
                       .perform(context))
                   ],
        condition=IfCondition(LaunchConfiguration("rviz"))
    )

    return [
        global_parameter_loader_launch,
        control_launch,
        localization_launch,
        map_launch,
        planning_launch,
        sensing_launch,
        system_launch,
        vehicle_launch,
        rviz2
    ]


def generate_launch_description():
    def add_launch_arg(name: str, default_value=None):
        return DeclareLaunchArgument(name, default_value=default_value)

    declared_arguments = []

    declared_arguments.append(add_launch_arg("map_path", "autoware_map/imola"))
    declared_arguments.append(add_launch_arg("map_yaml", "imola.yaml"))
    declared_arguments.append(add_launch_arg("vehicle_model", "f1tenth_vehicle"))
    declared_arguments.append(add_launch_arg("sensor_model", "f1tenth_sensor_kit"))
    declared_arguments.append(add_launch_arg("launch_vehicle", "true"))
    declared_arguments.append(add_launch_arg("launch_system", "false"))
    declared_arguments.append(add_launch_arg("launch_map", "false"))
    declared_arguments.append(add_launch_arg("launch_sensing", "true"))
    declared_arguments.append(add_launch_arg("launch_localization", "true"))
    declared_arguments.append(add_launch_arg("launch_planning", "true"))
    declared_arguments.append(add_launch_arg("launch_control", "true"))
    declared_arguments.append(add_launch_arg("use_sim_time", "true"))
    declared_arguments.append(add_launch_arg("vehicle_id", "default"))
    declared_arguments.append(add_launch_arg("launch_vehicle_interface", "true"))
    declared_arguments.append(add_launch_arg("lanelet2_map_file", "lanelet2_map.osm"))
    declared_arguments.append(add_launch_arg("system_run_mode", "online"))
    declared_arguments.append(add_launch_arg("vehicle_id", "default"))
    declared_arguments.append(add_launch_arg("launch_system_monitor", "true"))
    declared_arguments.append(add_launch_arg("launch_dummy_diag_publisher", "true"))
    declared_arguments.append(add_launch_arg("rviz", "true"))
    declared_arguments.append(add_launch_arg("rviz_config", "default.rviz"))

    return LaunchDescription([
        *declared_arguments,
        OpaqueFunction(function=launch_setup)
    ])
