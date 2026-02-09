import os

from moveit_configs_utils import MoveItConfigsBuilder

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# Define xacro mappings for the robot description file
def generate_launch_description():
    launch_arguments = {
        "robot_ip": "xxx.yyy.zzz.www",
        "use_fake_hardware": "true",
        "dof": "6",
        # "use_gazebo": "true"
    }

    # Load the robot configuration
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="kuka-omnirob-iisy",
            package_name="omnirob_iisy_moveit_config"
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
        parameters=[moveit_config.to_dict()]
    )

    rviz_config_arg = DeclareLaunchArgument(
        name="rviz_config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("omnirob_iisy_moveit_config"), "config", "moveit.rviz"]
        ),
        description="Path to the RViz config file"
    )
    rviz_base = LaunchConfiguration("rviz_config")
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("omnirob_iisy_moveit_config"), "config", rviz_base]
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

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="screen",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"]
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[{'use_sim_time': True,
                     'robot_description': moveit_config.robot_description}]
    )

    ros2_controller_path = os.path.join(
        get_package_share_directory("omnirob_iisy_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controller_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    omnirob_iisy_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("mpnp_simulation"),
                "launch",
                "omnirob_iisy.launch.py",
            )
        )
    )

    return LaunchDescription(
        [
            gazebo,
            rviz_config_arg,
            rviz_node,
            static_tf,
            robot_state_publisher,
            run_move_group_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            omnirob_iisy_arm_controller_spawner,
        ]
    )