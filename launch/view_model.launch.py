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
    declare_prefix_arg = DeclareLaunchArgument(
        "prefix",
        default_value="",
        description=(
            "A prefix for the names of joints, links, etc. in the robot model). "
            "E.g. 'base_link' will become 'cohort1_base_link' if prefix "
            "is set to 'cohort1'."
        ),
    )
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace under which to bring up nodes, topics, etc.",
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
    declare_use_rviz_config_template_arg = DeclareLaunchArgument(
        "use_rviz_config_template",
        default_value="true",
        description="If true, generate the RViz config from the specified RViz config template.",
    )
    declare_rviz_config_template_arg = DeclareLaunchArgument(
        "rviz_config_template",
        default_value=default_rviz_config_template_file,
        description="Path to the RViz config template file",
    )
    declare_rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=default_rviz_config_file,
        description="Path to the RViz config file",
    )
    declare_use_lidar_arg = DeclareLaunchArgument(
        "use_lidar",
        default_value="false",
        description="If true, include the lidar in the robot description",
    )
    declare_lidar_update_rate_arg = DeclareLaunchArgument(
        "lidar_update_rate",
        default_value="30",
        description="Set the update rate of the LiDAR sensor.",
    )

    # Launch Configurations
    model_package = LaunchConfiguration("model_package")
    model_file = LaunchConfiguration("model_file")
    prefix = LaunchConfiguration("prefix")
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_jsp = LaunchConfiguration("use_jsp")
    use_jsp_gui = LaunchConfiguration("use_jsp_gui")
    use_rviz = LaunchConfiguration("use_rviz")
    use_rviz_config_template = LaunchConfiguration("use_rviz_config_template")
    rviz_config_template = LaunchConfiguration("rviz_config_template")
    rviz_config = LaunchConfiguration("rviz_config")
    use_lidar = LaunchConfiguration("use_lidar")
    lidar_update_rate = LaunchConfiguration("lidar_update_rate")

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

    # Build the prefix with underscore.
    # This expression will evaluate to, for example, "cohort_" if
    # the prefix is "cohort", or to an empty string if prefix is empty.
    prefix_ = PythonExpression(
        ["'", prefix, "_' if '", prefix, "' else ''"]
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
                prefix_,
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
            declare_model_package_arg,
            declare_model_file_arg,
            declare_prefix_arg,
            declare_namespace_arg,
            declare_use_sim_time_arg,
            declare_use_jsp_arg,
            declare_use_jsp_gui_arg,
            declare_use_rviz_arg,
            declare_use_rviz_config_template_arg,
            declare_rviz_config_template_arg,
            declare_rviz_config_arg,
            declare_use_lidar_arg,
            declare_lidar_update_rate_arg,
            # Nodes
            push_namespace,
            rsp_node,
            jsp_node,
            jsp_gui_node,
            rviz_config_generator,
            rviz_node,
        ]
    )
