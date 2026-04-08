import rclpy
from rclpy import time
import pickle

from typing import List, Tuple

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Pose, TransformStamped, Point
from tf2_ros import TransformListener, Buffer

from mpnp_interfaces.msg import Object
from mpnp_interfaces.srv import Pick, Place, MoveBase

class TAMPInterface(Node):
    def __init__(self, plan: List[Tuple[str, List[str]]] | None = None):
        super().__init__('tamp_interface')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self.move_base_client = self.create_client(MoveBase, '/omnirob_controller/move_base')
        self.pick_client = self.create_client(Pick, '/koi_pick_place_controller/pick')
        self.place_client = self.create_client(Place, '/koi_pick_place_controller/place')
        self.plan = plan

        while not self.move_base_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for move_base service...')
        self.get_logger().info('move_base service available!')

        while not self.pick_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for pick service...')
        self.get_logger().info('pick service available!')

        while not self.place_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for place service...')
        self.get_logger().info('place service available!')

        self.execute_plan()

    def load_plan(self, plan: List[Tuple[str, List[str]]]):
        self.plan = plan
        self.get_logger().info(f'Loaded plan with {len(plan)} steps.')

    def execute_plan(self):
        if not self.plan:
            self.get_logger().error('No plan loaded!')
            return

        for act_name, args in self.plan:
            self.get_logger().info(f'Executing action: {act_name} with args: {args}')
            match act_name:
                case 'move':
                    self.execute_move_base(args)
                case 'pick':
                    self.execute_pick(args)
                case 'place':
                    self.execute_place(args)
                case _:
                    self.get_logger().error(f'Unknown action: {act_name}')

    def execute_move_base(self, args: List[str]):
        move_base_req = MoveBase.Request()
        target_pos_name = args[-1]
        target_pose_tf = self.tf_buffer.lookup_transform('world', target_pos_name, self.get_clock().now())

        target_pos = Point()
        target_pos.x = target_pose_tf.transform.translation.x
        target_pos.y = target_pose_tf.transform.translation.y
        target_pos.z = target_pose_tf.transform.translation.z
        move_base_req.target_position = target_pos

        future = self.move_base_client.call_async(move_base_req)
        future.add_done_callback(lambda f: self.get_logger().info(f'Move base {f.result().success} result: {f.result().message}'))
        rate = self.create_rate(1/5)
        while (not future.done() and rclpy.ok()):
            self.get_logger().info('Waiting for move_base result...')
            rate.sleep()

    def execute_pick(self, args: List[str]):
        pick_req = Pick.Request()
        self.get_logger().info(f'Executing pick with args: {args}')

    def execute_place(self, args: List[str]):
        place_req = Place.Request()
        self.get_logger().info(f'Executing place with args: {args}')

def load_plan_from_file(file_path: str) -> List[Tuple[str, List[str]]]:
    with open(file_path, 'rb') as f:
        plan = pickle.load(f)

    return plan

def main():
    plan = load_plan_from_file('src/object_rearrangement_ros2_sim/mpnp_simulation/config/plan.pkl')

    rclpy.init()
    tamp_interface = TAMPInterface(plan)
    # tamp_interface.load_plan(plan)
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