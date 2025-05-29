import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    GroupAction,
)
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    PathJoinSubstitution,
    PythonExpression,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_pkg = "arcs_cohort_rviz"
    description_pkg = get_package_share_directory("arcs_cohort_description")

    # Defaults
    default_model_file = "description/robot.urdf.xacro"
    default_rviz_config_file = os.path.join(
        get_package_share_directory(rviz_pkg), "rviz", "cohort_default.rviz"
    )
    default_log_level = "INFO"

    # Launch arguments
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace under which to bring up nodes, topics, etc.",
    )
    declare_prefix_arg = DeclareLaunchArgument("prefix",
        default_value="",
        description=(
            "A prefix for the names of joints, links, etc. in the robot model). "
            "E.g. 'base_link' will become 'cohort1_base_link' if prefix "
            "is set to 'cohort1'."
        ),
    )
    declare_model_package_arg = DeclareLaunchArgument(
        "model_package",
        default_value="arcs_cohort_description",
        description="Package with the robot model file",
    )
    declare_model_file_arg = DeclareLaunchArgument(
        "model_file",
        default_value=default_model_file,
        description="Path to URDF/Xacro file within model_package",
    )
    declare_rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=default_rviz_config_file,
        description="Path to the RViz config file",
    )
    declare_lidar_update_rate_arg = DeclareLaunchArgument(
        "lidar_update_rate",
        default_value="30",
        description="Set the update rate of the LiDAR sensor.",
    )
    declare_log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value=default_log_level,
        description="Set the log level for nodes.",
    )
    declare_use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation time if true"
    )
    declare_use_jsp_arg = DeclareLaunchArgument(
        "use_jsp",
        default_value="true",
        description="If true, launch the joint_state_publisher (CLI)",
    )
    declare_use_jsp_gui_arg = DeclareLaunchArgument(
        "use_jsp_gui",
        default_value="false",
        description="If true, launch the joint_state_publisher_gui",
    )
    declare_use_rviz_arg = DeclareLaunchArgument(
        "use_rviz", default_value="true", description="If true, launch RViz"
    )
    declare_use_lidar_arg = DeclareLaunchArgument(
        "use_lidar",
        default_value="false",
        description="If true, include the lidar in the robot description",
    )

    # Launch Configurations
    namespace = LaunchConfiguration("namespace")
    prefix = LaunchConfiguration("prefix")
    model_package = LaunchConfiguration("model_package")
    model_file = LaunchConfiguration("model_file")
    rviz_config = LaunchConfiguration("rviz_config")
    use_lidar = LaunchConfiguration("use_lidar")
    lidar_update_rate = LaunchConfiguration("lidar_update_rate")
    log_level = LaunchConfiguration("log_level")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_jsp = LaunchConfiguration("use_jsp")
    use_jsp_gui = LaunchConfiguration("use_jsp_gui")
    use_rviz = LaunchConfiguration("use_rviz")

    # Robot description from xacro
    robot_description = Command(
        [
            "xacro ",
            PathJoinSubstitution([FindPackageShare(model_package), model_file]),
            " prefix:=",
            prefix,
            " use_lidar:=",
            use_lidar,
            " lidar_update_rate:=",
            lidar_update_rate,
        ]
    )

    # Log info
    log_info = LogInfo(msg=['Model viewer launching with namespace: ', namespace, ', prefix: ', prefix])

    # Use PushRosNamespace to apply the namespace to all nodes below
    push_namespace = PushRosNamespace(namespace)

    # Robot State Publisher
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description, "use_sim_time": use_sim_time}
        ],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
    )

    # IfCondition for the CLI JSP: it should only run if use_jsp == 'true' **and** use_jsp_gui == 'false'
    jsp_node = Node(
        condition=IfCondition(
            PythonExpression(
                ["'true' == '", use_jsp, "' and 'true' != '", use_jsp_gui, "'"]
            )
        ),
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
    )

    # GUI JSP node: runs if use_jsp_gui == 'true'
    jsp_gui_node = Node(
        condition=IfCondition(use_jsp_gui),
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
    )

    # Include rviz.launch.py
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory(rviz_pkg),
                    "launch",
                    "rviz.launch.py",
                )
            ]
        ),
        condition=IfCondition(use_rviz),
        launch_arguments={
            "namespace": namespace,
            "prefix": prefix,
            "rviz_config": rviz_config,
            "log_level": log_level,
        }.items(),
    )

    return LaunchDescription(
        [
            # Declare launch arguments
            declare_model_package_arg,
            declare_model_file_arg,
            declare_prefix_arg,
            declare_namespace_arg,
            declare_rviz_config_arg,
            declare_lidar_update_rate_arg,
            declare_log_level_arg,
            declare_use_sim_time_arg,
            declare_use_jsp_arg,
            declare_use_jsp_gui_arg,
            declare_use_rviz_arg,
            declare_use_lidar_arg,
            # Log
            log_info,
            # Nodes
            GroupAction([
                push_namespace,
                rsp_node,
                jsp_node,
                jsp_gui_node,
            ]),
            # Launchers
            rviz_launch,
        ]
    )
