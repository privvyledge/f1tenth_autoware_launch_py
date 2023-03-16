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
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    pkg_prefix = FindPackageShare("f1tenth_launch")
    nav2_pkg_prefix = FindPackageShare("nav2_bringup")

    localization_param_file = PathJoinSubstitution(
        [pkg_prefix, "config/localization/amcl.param.yaml"])

    nav2_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([
                nav2_pkg_prefix, "launch", "localization_launch.py"
            ]),
        ),
        launch_arguments={
            "params_file": localization_param_file,
            "map": LaunchConfiguration("map_yaml"),
        }.items()
    )

    gyro_odometer_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([
                FindPackageShare("gyro_odometer"), "launch", "gyro_odometer.launch.xml"
            ]),
        ),
        launch_arguments={
            "input_vehicle_twist_with_covariance_topic": "/sensing/vehicle_velocity_converter/twist_with_covariance",
            "input_imu_topic": "/sensing/vesc/imu",
            "output_twist_with_covariance_topic": "/localization/twist_estimator/twist_with_covariance",
            "output_twist_with_covariance_raw_topic": "/localization/twist_estimator/twist_with_covariance_raw",
            "output_twist_topic": "/localization/twist_estimator/twist",
            "output_twist_raw_topic": "/localization/twist_estimator/twist_raw"
        }.items()
    )

    twist_to_odom_node = Node(
        name="twist_to_odom_node",
        namespace="localization",
        package="twist_to_odom",
        executable="twist_to_odom_node_exe",
        parameters=[
            {
                "publish_tf": True,
            }
        ],
        remappings=[
            ("twist_with_covariance", "/localization/twist_estimator/twist_with_covariance"),
            ("odometry", "/localization/odometry")
        ],
        output="screen",
    )

    # map_to_odom_tf_publisher = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="static_map_to_odom_tf_publisher",
    #     output="screen",
    #     arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "map", "odom"],
    # )

    return [
        gyro_odometer_launch,
        twist_to_odom_node,
        nav2_localization,
        # map_to_odom_tf_publisher
    ]


def generate_launch_description():
    declared_arguments = []

    def add_launch_arg(name: str, default_value=None):
        declared_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value)
        )

    add_launch_arg("map_yaml", "imola.yaml")

    return LaunchDescription([
        *declared_arguments,
        OpaqueFunction(function=launch_setup)
    ])
