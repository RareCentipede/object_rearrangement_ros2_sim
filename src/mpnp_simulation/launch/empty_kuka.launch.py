import os

from enum import Enum

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, RegisterEventHandler, ExecuteProcess
from launch.substitutions import PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description() -> LaunchDescription:
    world = 'empty'
    robot = 'kuka-omnirob-lwrs'

    package_dir = get_package_share_directory('mpnp_simulation')
    world_path = PathJoinSubstitution([package_dir, 'worlds', world + '.sdf'])
    if robot != 'kuka-omnirob-lwrs':
        model_path = os.path.join(package_dir, 'models', robot, 'urdf', robot + '.urdf')
    else:
        model_path = os.path.join(package_dir, 'models', robot, 'model.urdf')

    with open(model_path, 'r') as f:
        robot_desc = f.read()

    ros_gz_sim_pkg_path = FindPackageShare('ros_gz_sim')
    gz_sim_launch_path = PathJoinSubstitution([ros_gz_sim_pkg_path, 'launch', 'gz_sim.launch.py'])
    gz_model_launch_path = PathJoinSubstitution([ros_gz_sim_pkg_path, 'launch', 'gz_spawn_model.launch.py'])

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
            'file': model_path,
            'entity_name': robot,
            'x': '0.0',
            'y': '0.0',
            'z': '0.0',
            'roll': '0.0',
            'pitch': '0.0',
            'yaw': '0.0',}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True,
                     'robot_description': robot_desc}]
    )

    models = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.pathsep.join([
            package_dir,
            'models',
            os.environ.get('GZ_SIM_RESOURCE_PATH', '')
        ])
    )

    ld = LaunchDescription(
        [
            models,
            world_launch_description,
            spawn_robot,
            robot_state_publisher,
        ]
    )

    return ld