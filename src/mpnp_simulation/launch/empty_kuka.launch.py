from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, RegisterEventHandler, ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description() -> LaunchDescription:
    world = 'empty'
    robot = 'kuka_omnirob'

    package_dir = get_package_share_directory('mpnp_simulation')
    world_path = PathJoinSubstitution([package_dir, 'worlds', world + '.sdf'])

    ros_gz_sim_pkg_path = get_package_share_directory('ros_gz_sim')
    gz_launch_path = os.path.join(ros_gz_sim_pkg_path, 'launch', 'gz_sim.launch.py')

    models = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(package_dir, 'models')
    )

    world_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={
            'gz_args': [world_path],
            'on_exit_shutdown': 'True'
        }.items(),
    )

    ld = LaunchDescription(
        [
            models,
            world_launch_description,
        ]
    )

    return ld