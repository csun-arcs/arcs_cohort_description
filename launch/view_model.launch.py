import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    PathJoinSubstitution,
    PythonExpression,
)
from launch.conditions import IfCondition
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_description = get_package_share_directory("arcs_cohort_description")
    default_model_file = "description/robot.urdf.xacro"
    default_rviz_config_template_file = os.path.join(
        pkg_description, "rviz_config", "robot_model.rviz.template"
    )
    default_rviz_config_file = os.path.join(
        pkg_description, "rviz_config", "robot_model.rviz"
    )

    # Launch arguments
    declare_model_package_cmd = DeclareLaunchArgument(
        "model_package",
        default_value="arcs_cohort_description",
        description="Package with the robot model file",
    )
    declare_model_file_cmd = DeclareLaunchArgument(
        "model_file",
        default_value=default_model_file,
        description="Path to URDF/Xacro file within model_package",
    )
    declare_robot_name_cmd = DeclareLaunchArgument(
        "robot_name",
        default_value="",
        description=(
            "Name of the robot (specifying this will add the "
            "robot name prefix to joints, links, etc. in the robot model)."
        ),
    )
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace under which to bring up nodes, topics, etc.",
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation time if true"
    )
    declare_use_jsp_cmd = DeclareLaunchArgument(
        "use_jsp",
        default_value="true",
        description="If true, launch the joint_state_publisher (CLI)",
    )
    declare_use_jsp_gui_cmd = DeclareLaunchArgument(
        "use_jsp_gui",
        default_value="false",
        description="If true, launch the joint_state_publisher_gui",
    )
    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz", default_value="true", description="If true, launch RViz"
    )
    declare_use_rviz_config_template_cmd = DeclareLaunchArgument(
        "use_rviz_config_template",
        default_value="true",
        description="If true, generate the RViz config from the specified RViz config template.",
    )
    declare_rviz_config_template_cmd = DeclareLaunchArgument(
        "rviz_config_template",
        default_value=default_rviz_config_template_file,
        description="Path to the RViz config template file",
    )
    declare_rviz_config_cmd = DeclareLaunchArgument(
        "rviz_config",
        default_value=default_rviz_config_file,
        description="Path to the RViz config file",
    )
    declare_use_lidar_cmd = DeclareLaunchArgument(
        "use_lidar",
        default_value="false",
        description="If true, include the lidar in the robot description",
    )

    # Launch Configurations
    model_package = LaunchConfiguration("model_package")
    model_file = LaunchConfiguration("model_file")
    robot_name = LaunchConfiguration("robot_name")
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_jsp = LaunchConfiguration("use_jsp")
    use_jsp_gui = LaunchConfiguration("use_jsp_gui")
    use_rviz = LaunchConfiguration("use_rviz")
    use_rviz_config_template = LaunchConfiguration("use_rviz_config_template")
    rviz_config_template = LaunchConfiguration("rviz_config_template")
    rviz_config = LaunchConfiguration("rviz_config")
    use_lidar = LaunchConfiguration("use_lidar")

    # Compute the robot prefix only if a robot name is provided
    # This expression will evaluate to, for example, "cohort_" if
    # robot_name is "cohort", or to an empty string if robot_name is empty.
    robot_prefix = PythonExpression(
        ["'", robot_name, "_' if '", robot_name, "' else ''"]
    )
    # Compute the prefix argument only if a robot_name/robot_prefix is provided.
    # This expression will evaluate to, for example, "prefix:=cohort_" if
    # robot_prefix is "cohort_", or to an empty string if robot_prefix is empty.
    robot_prefix_arg = PythonExpression(
        ["('prefix:=' + '", robot_prefix, "') if '", robot_prefix, "' else ''"]
    )

    # Robot description from Xacro, including the conditional robot name prefix.
    robot_description = Command(
        [
            "xacro ",
            PathJoinSubstitution([FindPackageShare(model_package), model_file]),
            " ",
            robot_prefix_arg,
            " use_lidar:=",
            use_lidar,
        ]
    )

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

    # Generate RViz config from template.
    # The robot prefix will be substituted into the RViz config template in
    # place of the ARCS_COHORT_PREFIX variable and the namespace will be
    # substituted in place of ARCS_COHORT_NAMESPACE.
    #
    # NOTE: We should probably change this approach later.  It's a neat trick,
    # but might not be manageable/scaleable.  Using fixed RViz configurations
    # for different robot/world scenarios is probably a more robust approach.
    # This type of dynamic RViz config generation could still be useful in the
    # early stages of project development to test namespacing, prefixing, etc.
    #
    namespace_env_var = PythonExpression(
        ["'/", namespace, "' if '", namespace, "' else ''"]
    )
    rviz_config_generator = ExecuteProcess(
        condition=IfCondition(use_rviz_config_template),
        cmd=[
            [
                "ARCS_COHORT_PREFIX='",
                robot_prefix,
                "' ",
                "ARCS_COHORT_NAMESPACE='",
                namespace_env_var,
                "' ",
                "envsubst < ",
                rviz_config_template,
                " > ",
                rviz_config,
            ]
        ],
        shell=True,
        output="screen",
    )

    # RViz node
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
    )

    return LaunchDescription(
        [
            # Declare launch arguments
            declare_model_package_cmd,
            declare_model_file_cmd,
            declare_robot_name_cmd,
            declare_namespace_cmd,
            declare_use_sim_time_cmd,
            declare_use_jsp_cmd,
            declare_use_jsp_gui_cmd,
            declare_use_rviz_cmd,
            declare_use_rviz_config_template_cmd,
            declare_rviz_config_template_cmd,
            declare_rviz_config_cmd,
            declare_use_lidar_cmd,
            # Nodes
            push_namespace,
            rsp_node,
            jsp_node,
            jsp_gui_node,
            rviz_config_generator,
            rviz_node,
        ]
    )
