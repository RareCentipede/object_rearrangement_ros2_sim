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

    omnirob_iisy_vgc10_moveit_pkg_share = FindPackageShare("omnirob_iisy_vgc10_moveit_config")
    mpnp_dir = os.path.join(get_package_prefix("mpnp_simulation"), "share", "mpnp_simulation")

    box_path = os.path.join(mpnp_dir, "models", "box.sdf")
    if not os.path.exists(box_path):
        raise FileNotFoundError(f"Box URDF not found at {box_path}. Please ensure the file exists.")

    with open(box_path, 'r') as box_file:
        box_urdf_content = box_file.read()

    # Append world path to GZ_SIM_RESOURCE_PATH
    append_gz_env = (AppendEnvironmentVariable(
                    name="GZ_SIM_RESOURCE_PATH",
                    value=mpnp_dir
                    )
                  )

    # Get URDF via xacro
    robot_description_content = Command(
            [FindExecutable(name="xacro"), 
             ' ',
             PathJoinSubstitution(
                 [omnirob_iisy_vgc10_moveit_pkg_share, 
                  "config", "kuka_omnirob_iisy_vgc10.urdf.xacro"]
                 )
             ]
        )
    robot_description = {'robot_description': robot_description_content}

    # Run all nodes
    robot_controllers = PathJoinSubstitution(
        [
            omnirob_iisy_vgc10_moveit_pkg_share,
            "config",
            "ros2_controllers.yaml"
        ]
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description,
                    {"use_sim_time": True}]
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            '-topic', '/robot_description',
            '-name', 'omnirob_iisy_vgc10',
            '-allow_renaming', 'true'
        ]
    )

    spawn_box = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            '-file', box_path,   # ← spawn directly from sdf file
            '-name', 'box',
            '-x', '2', '-y', '2', '-z', '0.05',  # ← set pose here instead of static tf
            '-allow_renaming', 'true'
            ]
    )

    box_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['2', '2', '0.05', '0', '0', '0', 'world', 'box/base_link'],
        parameters=[{"use_sim_time": True}],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager",
                   "--param-file", robot_controllers],
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller",
                   "--controller-manager", "/controller_manager",
                   "--param-file", robot_controllers],
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    omnirob_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["omnirob_controller",
                   "--controller-manager", "/controller_manager",
                   "--param-file", robot_controllers],
        output="screen",
        parameters=[{"use_sim_time": True}]
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
        parameters=[{"use_sim_time": True}]
    )

    ros_gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )        ),
        launch_arguments=[('gz_args', [' -r -v 1 worlds/empty.sdf'])])

    spawner_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
    joint_state_broadcaster_spawner_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )
    arm_controller_spawner_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[omnirob_controller_spawner],
        )
    )

    omnirob_controller_node = Node(
        package="koi_controller",
        executable="omnirob_controller",
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    iisy_arm_controller_node = Node(
        package="koi_controller",
        executable="iisy_arm_controller",
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    ld.add_action(append_gz_env)
    ld.add_action(bridge)
    ld.add_action(ros_gz_sim_launch)
    ld.add_action(spawner_event_handler)
    ld.add_action(arm_controller_spawner_event_handler)
    ld.add_action(joint_state_broadcaster_spawner_event_handler)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(gz_spawn_entity)

    ld.add_action(spawn_box)
    ld.add_action(box_tf_node)
    ld.add_action(omnirob_controller_node)
    # ld.add_action(iisy_arm_controller_node)

    return ld