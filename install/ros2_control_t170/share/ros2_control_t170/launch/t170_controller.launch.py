from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("gui", default_value="true", description="Start RViz2 automatically with this launch file."),
    ]

    # 加载 URDF 文件
    urdf_path = os.path.join(get_package_share_directory('ros2_control_t170'), 'urdf', 'ti5.urdf')
    urdf = open(urdf_path).read()
    robot_description = {"robot_description": urdf}

    # 控制器配置文件
    robot_controllers = PathJoinSubstitution([FindPackageShare("ros2_control_t170"), "config", "t170_controller.yaml"])
    rviz_config_file = PathJoinSubstitution([FindPackageShare("ros2_control_t170"), "rviz", "view_t170.rviz"])
    teleop_config_file = PathJoinSubstitution([FindPackageShare("ros2_control_t170"), "config", "teleop_config.yaml"])

    # 控制器管理节点
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        remappings=[("~/robot_description", "/robot_description")],
        output="screen",
    )

    # 机器人状态发布节点
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # RViz2 节点（可选）
    gui = LaunchConfiguration("gui")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",  # 改为 screen 以便调试
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    # 关节状态广播器
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # 机器人控制器
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["t170_controller", "-c", "/controller_manager"],
        output="screen",
    )

    # 手柄控制节点
    teleop_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy_node",
        parameters=[teleop_config_file],
        remappings=[("/cmd_vel", "/cmd_vel")],
        output="screen",
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        parameters=[{"dev": "/dev/input/js0"}],
    )

    joy_to_float32_node = Node(
        package="ros2_control_t170",
        executable="joy_to_float32",
        name="joy_to_float32_node",
        output="screen",
    )

    # 事件处理器
    delay_rviz = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=joint_state_broadcaster_spawner, on_exit=[rviz_node])
    )
    delay_joint_state = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=robot_controller_spawner, on_exit=[joint_state_broadcaster_spawner])
    )
    delay_teleop = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[teleop_node, joy_node, joy_to_float32_node],
        )
    )

    # 节点列表
    nodes = [
        control_node,
        robot_state_pub_node,
        robot_controller_spawner,
        delay_rviz,
        delay_joint_state,
        delay_teleop,
    ]

    return LaunchDescription(declared_arguments + nodes)