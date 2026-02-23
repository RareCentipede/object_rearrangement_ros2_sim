import os

from moveit_configs_utils import MoveItConfigsBuilder

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch_ros.actions import Node

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# Define xacro mappings for the robot description file
def generate_launch_description():
    launch_arguments = {
        "robot_ip": "xxx.yyy.zzz.www",
        "use_fake_hardware": "false",
        "dof": "6",
        "use_gazebo": "true"
    }

    # Load the robot configuration
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="kuka_omnirob_iisy_vgc10",
            package_name="omnirob_iisy_vgc10_moveit_config"
        )
        .robot_description(mappings=launch_arguments) #type: ignore
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(
            pipelines=["ompl", "stomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(),
                        {"publish_planning_scene_hz": 30.0},
                        {"allow_trajectory_execution": True},
                        {"use_sim_time": True},
                        {"publish_planning_scene": True},
                        {"publish_state_updates": True},
                        {"publish_transforms_updates": True}
                    ]
    )

    rviz_config_arg = DeclareLaunchArgument(
        name="rviz_config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("omnirob_iisy_vgc10_moveit_config"), "config", "moveit.rviz"]
        ),
        description="Path to the RViz config file"
    )
    rviz_base = LaunchConfiguration("rviz_config")
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("omnirob_iisy_vgc10_moveit_config"), "config", rviz_base]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits
        ]
    )

    omnirob_iisy_vgc10_gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("mpnp_simulation"), "launch", "omnirob_iisy_vgc10.launch.py"]
            )
        )
    )

    return LaunchDescription(
        [
            rviz_config_arg,
            rviz_node,
            run_move_group_node,
            omnirob_iisy_vgc10_gz_launch,
        ]
    )