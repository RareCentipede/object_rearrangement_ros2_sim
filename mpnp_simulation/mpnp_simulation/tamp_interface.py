import rclpy
import pickle

from typing import List, Tuple

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Pose, TransformStamped
from tf2_ros import TransformListener, Buffer

from mpnp_interfaces.msg import Object
from mpnp_interfaces.srv import Pick, Place, MoveBase

class TAMPInterface(Node):
    def __init__(self):
        super().__init__('tamp_interface')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.move_base_client = self.create_client(MoveBase, '/omnirob_controller/move_base')
        self.pick_client = self.create_client(Pick, '/koi_pick_place_controller/pick')
        self.place_client = self.create_client(Place, '/koi_pick_place_controller/place')
        self.plan = None

        while not self.move_base_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for move_base service...')
        self.get_logger().info('move_base service available!')

        while not self.pick_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for pick service...')
        self.get_logger().info('pick service available!')

        while not self.place_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for place service...')
        self.get_logger().info('place service available!')

    def load_plan(self, plan: List[Tuple[str, List[str]]]):
        self.plan = plan
        self.get_logger().info(f'Loaded plan with {len(plan)} steps.')

def load_plan_from_file(file_path: str) -> List[Tuple[str, List[str]]]:
    with open(file_path, 'rb') as f:
        plan = pickle.load(f)

    return plan

def main():
    plan = load_plan_from_file('src/object_rearrangement_ros2_sim/mpnp_simulation/config/plan.pkl')

    rclpy.init()
    tamp_interface = TAMPInterface()
    tamp_interface.load_plan(plan)
    executor = MultiThreadedExecutor()
    executor.add_node(tamp_interface)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        tamp_interface.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()