import rclpy
import yaml
import sys

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
from tf2_ros import StaticTransformBroadcaster

from gz.transport13 import Node as GzNode
from gz.msgs10 import pose_pb2, pose_v_pb2

class WorldManager(Node):
    def __init__(self, problem_name: str):
        super().__init__('world_manager')

        self.config_path = 'src/object_rearrangement_ros2_sim/mpnp_simulation/config/problem_configs'
        self.problem_path = f'{self.config_path}/{problem_name}'
        self.init_config = yaml.safe_load(open(f'{self.problem_path}/init.yaml', 'r'))
        self.goal_config = yaml.safe_load(open(f'{self.problem_path}/goal.yaml', 'r'))

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        self.get_logger().info(f'Loaded problem: {problem_name}')
        self.get_logger().info(f'Initial configuration: {self.init_config}')
        self.get_logger().info(f'Goal configuration: {self.goal_config}')

        self.gz_node = GzNode()

def main():
    args = sys.argv[1:]
    rclpy.init()
    world_manager = WorldManager(args[0] if args else 'basic')
    executor = MultiThreadedExecutor()
    executor.add_node(world_manager)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        world_manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()