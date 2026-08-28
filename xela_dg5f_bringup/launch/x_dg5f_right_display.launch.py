from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "description_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("xela_dg5f_bringup"), "urdf", "x_dg5f_right_standalone.xacro"]
            ),
            description="URDF/XACRO description file (absolute path) with the robot.",
        ),
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("xela_dg5f_bringup"), "config", "x_dg5f_right_display.rviz"]
            ),
            description="RViz config file (absolute path) to use when launching rviz.",
        ),
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Use joint_state_publisher_gui to manually drive the joints.",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Launch RViz2.",
        ),
        DeclareLaunchArgument(
            "taxels",
            default_value="0",
            description=(
                "Individual XELA taxel-dot markers (124 total) inside each sensor "
                "housing. Default 0 matches what MoveIt Pro's own robot_description "
                "actually loads for ur7e_xdg5f_atag_right_sim (sensor housings are "
                "always present regardless of this flag). Set to 1 to preview the "
                "full taxel-dot model."
            ),
        ),
        DeclareLaunchArgument(
            "sensor_collision",
            default_value="0",
            description="Whether taxel-dot markers (when taxels=1) get <collision> geometry.",
        ),
    ]

    description_file = LaunchConfiguration("description_file")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    show_gui = LaunchConfiguration("gui")
    show_rviz = LaunchConfiguration("rviz")
    taxels = LaunchConfiguration("taxels")
    sensor_collision = LaunchConfiguration("sensor_collision")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            description_file,
            " taxels:=",
            taxels,
            " sensor_collision:=",
            sensor_collision,
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_publisher_node = Node(
        condition=UnlessCondition(show_gui),
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[robot_description],
    )

    joint_state_publisher_gui_node = Node(
        condition=IfCondition(show_gui),
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
    )

    rviz_node = Node(
        condition=IfCondition(show_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    nodes_to_start = [
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
