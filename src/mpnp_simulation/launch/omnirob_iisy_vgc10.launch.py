import os

from ament_index_python import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    iisy_vgc10_moveit_pkg_share = FindPackageShare("iisy_vgc10_moveit_config")
    mpnp_simulation_pkg_share = FindPackageShare("mpnp_simulation")
    omnirob_description_pkg_share = FindPackageShare("omnirob_description")

    # Get URDF via xacro
    robot_description_content = Command(
            [FindExecutable(name="xacro"), 
             ' ',
             PathJoinSubstitution(
                 [mpnp_simulation_pkg_share, 
                  "models", "kuka_omnirob_iisy_vgc10.urdf.xacro"]
                 )
             ]
        )
    robot_description = {'robot_description': robot_description_content}

    # Run all nodes
    iisy_vgc10_controllers = PathJoinSubstitution(
        [
            iisy_vgc10_moveit_pkg_share,
            "config",
            "ros2_controllers.yaml"
        ]
    )

    omnirob_controllers = PathJoinSubstitution(
        [
            omnirob_description_pkg_share,
            "config",
            "ros2_controllers.yaml"
        ]
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description]
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            '-topic', 'robot_description',
            '-name', 'omnirob-iisy-vgc10',
            '-allow_renaming', 'true'
        ]
    )

    arm_joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager",
                   "--param-file", iisy_vgc10_controllers],
        output="screen",
    )

    omnirob_joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager",
                   "--param-file", omnirob_controllers],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller",
                   "--controller-manager", "/controller_manager",
                   "--param-file", iisy_vgc10_controllers],
        output="screen",
    )

    omnirob_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["omnirob_controller_with_rotation",
                   "--controller-manager", "/controller_manager",
                   "--param-file", omnirob_controllers],
        output="screen",
    )

    # Bridge
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
    )

    ros_gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )        ),
        launch_arguments=[('gz_args', [' -r -v 1 empty.sdf'])])

    spawner_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[arm_joint_state_broadcaster_spawner],
        )
    )
    omnirob_joint_state_broadcaster_spawner_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_joint_state_broadcaster_spawner,
            on_exit=[omnirob_joint_state_broadcaster_spawner],
        )
    )
    arm_controller_spawner_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[omnirob_controller_spawner],
        )
    )

    ld.add_action(ros_gz_sim_launch)
    ld.add_action(spawner_event_handler)
    ld.add_action(arm_controller_spawner_event_handler)
    ld.add_action(omnirob_joint_state_broadcaster_spawner_event_handler)
    ld.add_action(bridge)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(gz_spawn_entity)

    return ld