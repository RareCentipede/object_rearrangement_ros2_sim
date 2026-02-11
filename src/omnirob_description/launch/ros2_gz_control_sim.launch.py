import os

from ament_index_python import get_package_prefix
from launch import LaunchDescription
from launch.event_handlers import OnProcessExit
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, AppendEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ld = LaunchDescription()
    pkg_share = FindPackageShare(package='omnirob_description')
    default_model_path = PathJoinSubstitution([pkg_share, 'urdf', 'omnirob.urdf.xacro'])

    models = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(get_package_prefix('omnirob_description'), 'share')
    )

    robot_description_content = Command(
            [FindExecutable(name="xacro"), 
             ' ',
             default_model_path
             ]
        )
    robot_description = {'robot_description': robot_description_content}

    # Run ROS2 control node
    omnirob_controllers = PathJoinSubstitution(
        [
            pkg_share,
            "config",
            "omnirob_ros2_controllers.yaml"
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
            '-name', 'omnirob',
            '-allow_renaming', 'true'
        ]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager",
                   "--param-file", omnirob_controllers],
        output="screen"
    )

    omnirob_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["omnirob_controller_with_rotation",
                   "--controller-manager", "/controller_manager",
                   "--param-file", omnirob_controllers],
        output="screen"
    )

    # Bridge
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"
        ]
    )

    ros_gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={"gz_args": "-r empty.sdf"}.items()
    )

    robot_spawner_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[joint_state_broadcaster_spawner]
        )
    )

    joint_state_broadcaster_spawner_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[omnirob_controller_spawner]
        )
    )

    ld.add_action(models)
    ld.add_action(ros_gz_sim_launch)
    ld.add_action(robot_spawner_event_handler)
    ld.add_action(joint_state_broadcaster_spawner_event_handler)
    ld.add_action(bridge)
    ld.add_action(gz_spawn_entity)
    ld.add_action(robot_state_publisher_node)

    return ld