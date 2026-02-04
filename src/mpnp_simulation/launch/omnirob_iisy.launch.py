import os
import subprocess

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.substitutions import PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.launch_context import LaunchContext

def generate_launch_description() -> LaunchDescription:
    world = 'empty'
    robot = 'kuka-omnirob-iisy'

    mpnp_package_dir = FindPackageShare('mpnp_simulation')
    world_path = PathJoinSubstitution([mpnp_package_dir, 'worlds', world + '.sdf'])
    robot_xacro = PathJoinSubstitution([mpnp_package_dir, 'models', robot, robot + '.urdf.xacro'])
    robot_model_path = PathJoinSubstitution([mpnp_package_dir, 'models', robot, robot + '.urdf'])

    robot_xacro_path_str = robot_xacro.perform(LaunchContext())
    robot_model_path_str = robot_model_path.perform(LaunchContext())
    # Generate URDF from XACRO
    subprocess.run(['xacro', robot_xacro_path_str, '-o', robot_model_path_str])

    with open(robot_model_path_str, 'r') as file:
        robot_description = file.read()

    ros_gz_sim_pkg_path = FindPackageShare('ros_gz_sim')
    gz_sim_launch_path = PathJoinSubstitution([ros_gz_sim_pkg_path, 'launch', 'gz_sim.launch.py'])
    gz_model_launch_path = PathJoinSubstitution([ros_gz_sim_pkg_path, 'launch', 'gz_spawn_model.launch.py'])

    models = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(get_package_prefix('mpnp_simulation'), 'share')
    )

    world_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch_path),
        launch_arguments={
            'gz_args': [world_path],
            'on_exit_shutdown': 'True'
        }.items(),
    )

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_model_launch_path),
        launch_arguments={
            'world': world,
            'file': robot_model_path_str,
            'entity_name': robot,
            'x': '0.0',
            'y': '0.0',
            'z': '0.0',
            'roll': '0.0',
            'pitch': '0.0',
            'yaw': '0.0',}.items(),
    )

    gazebo_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True,
                     'robot_description': robot_description}]
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="log",
    )

    ld = LaunchDescription(
        [
            models,
            gazebo_bridge,
            world_launch_description,
            spawn_robot,
            robot_state_publisher,
            # joint_state_publisher_gui,
        ]
    )

    return ld