import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    pkg_path = os.path.join(get_package_share_directory("arcs_cohort_description"))
    xacro_file = os.path.join(pkg_path, "description", "robot.urdf.xacro")
    robot_description_config = xacro.process_file(xacro_file)
    rviz_config_file = os.path.join(pkg_path, "rviz_config", "robot_model.rviz")

    params = {
        "robot_description": robot_description_config.toxml(),
        "use_sim_time": use_sim_time,
    }
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[params],
    )

    # node_joint_state_publisher = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     output='screen'
    # )

    node_rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use sim time if true",
            ),
            node_robot_state_publisher,
            # node_joint_state_publisher,
            node_rviz2,
        ]
    )
