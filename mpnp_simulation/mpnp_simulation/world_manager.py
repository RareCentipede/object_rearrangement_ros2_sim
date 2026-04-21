import rclpy
import sys
import numpy as np

from typing import Tuple, cast
from yaml import safe_load

from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Pose, TransformStamped
from tf2_geometry_msgs import PoseStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster, Buffer, TransformListener

from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V #type: ignore
from gz.msgs10.entity_factory_pb2 import EntityFactory #type: ignore
from gz.msgs10.boolean_pb2 import Boolean #type: ignore

from mpnp_interfaces.msg import Plan, Surface, Block
from mpnp_interfaces.srv import PlanConstructionTask, ExecutePlan

from mpnp_simulation.scripts.generate_polyhedrals import generate_diced_block, compute_base_positions

class WorldManager(Node):
    def __init__(self, problem_name: str):
        super().__init__('world_manager')
        self.gz_node = GzNode()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self.config_path = 'src/object_rearrangement_ros2_sim/mpnp_simulation/config/problem_configs'
        self.box_path = 'src/object_rearrangement_ros2_sim/mpnp_simulation/models/box.sdf'
        self.box_size = 0.3

        self.problem_name = problem_name
        self.problem_path = f'{self.config_path}/{problem_name}'
        self.init_config = safe_load(open(f'{self.problem_path}/init.yaml', 'r'))
        self.goal_config = safe_load(open(f'{self.problem_path}/goal.yaml', 'r'))
        self.get_logger().info(f'Loaded problem: {problem_name}')

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.poses_dict = {}
        self.poses_list = []
        self.objs = []
        self.blocks = {}
        self.base_positions = []

        self.define_objs_poses_init_blocks()
        self.create_polyhedral_blocks()

        self.robot_pose_pub = self.create_publisher(Pose, '/omnirob_iisy_vgc10/pose', 10, callback_group=ReentrantCallbackGroup())
        self.gz_node.subscribe(Pose_V, '/world/empty/pose/info', self.gz_pose_callback)

        self.publish_position_tfs()
        self.publish_surfaces_and_base_positions()

        self.task_planner_client = self.create_client(PlanConstructionTask, '/tamp/plan_construction_task')
        while not self.task_planner_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for plan_construction_task service...')
        self.get_logger().info('plan_construction_task service online!')

        self.plan_executor_client = self.create_client(ExecutePlan, '/tamp/execute_plan')
        while not self.plan_executor_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for execute_plan service...')
        self.get_logger().info('execute_plan service online!')

        self.request_plan()

    def define_objs_poses_init_blocks(self):
        idx = 1

        for obj_name, info in self.init_config.items():
            if obj_name != 'robot':
                self.objs.append(obj_name)

            pose = Pose()
            pose.position.x = info['position'][0]
            pose.position.y = info['position'][1]
            pose.position.z = (info['position'][2] + 1) * self.box_size / 2.0
            pose.orientation.x = info['orientation'][0]
            pose.orientation.y = info['orientation'][1]
            pose.orientation.z = info['orientation'][2]
            pose.orientation.w = 1.0

            self.poses_list.append([pose.position.x, pose.position.y, pose.position.z])

            pose_name = 'p' + str(idx)
            self.poses_dict[pose_name] = pose
            idx += 1

            block = Block()
            block.name = obj_name
            block.init_pose = pose
            self.blocks[obj_name] = block

            self.spawn_object(obj_name, pose)

        for obj_name, info in self.goal_config.items():
            pos = info['position']
            pose_name, pose = self.find_pose_from_position(pos)

            if pose_name not in self.poses_dict:
                self.poses_list.append(pos)
                orientation = info.get('orientation', [0.0, 0.0, 0.0])
                pose.orientation.x, pose.orientation.y, pose.orientation.z = orientation[0], orientation[1], orientation[2]
                self.poses_dict[pose_name] = pose

                block = self.blocks[obj_name]
                block.goal_pose = pose
                self.blocks.update({obj_name: block})

    def spawn_object(self, obj_name: str, pose: Pose):
        spawn_obj_req = EntityFactory()
        spawn_obj_req.sdf = open(f'src/object_rearrangement_ros2_sim/mpnp_simulation/models/box.sdf', 'r').read()
        spawn_obj_req.pose.position.x = pose.position.x
        spawn_obj_req.pose.position.y = pose.position.y
        spawn_obj_req.pose.position.z = pose.position.z
        spawn_obj_req.pose.orientation.x = pose.orientation.x
        spawn_obj_req.pose.orientation.y = pose.orientation.y
        spawn_obj_req.pose.orientation.z = pose.orientation.z
        spawn_obj_req.pose.orientation.w = pose.orientation.w
        spawn_obj_req.name = obj_name
        spawn_obj_req.allow_renaming = False

        spawn_result, spawn_response = self.gz_node.request('/world/empty/create',
                                                            spawn_obj_req, EntityFactory, Boolean, 5000)

        if spawn_result:
            if spawn_response.data:
                self.get_logger().info(f'Successfully spawned {obj_name}')
            else:
                self.get_logger().error(f'Failed to spawn {obj_name}: Service returned False')
        else:
            self.get_logger().error(f'Failed to spawn {obj_name}: Service call failed')

    def create_polyhedral_blocks(self):
        for obj_name in self.objs:
            surfaces = []
            _, _, clean_faces = generate_diced_block(self.box_size/2, 14)
            base_placements, valid_centers, valid_normals = compute_base_positions(clean_faces)

            block = self.blocks[obj_name]
            for c, n in zip(valid_centers, valid_normals):
                surfaces.append(Surface(
                    center=c,
                    normal=n
                ))
            block.surfaces = surfaces
            block.base_positions = base_placements
            self.blocks.update({obj_name: block})
            self.base_positions.append(base_placements)

    def publish_position_tfs(self):
        for pose_name in list(self.poses_dict.keys()):
            pose = self.poses_dict[pose_name]
            tf_pose = TransformStamped()
            tf_pose.header.stamp = self.get_clock().now().to_msg()
            tf_pose.header.frame_id = 'world'
            tf_pose.child_frame_id = pose_name
            tf_pose.transform.translation.x = pose.position.x
            tf_pose.transform.translation.y = pose.position.y
            tf_pose.transform.translation.z = pose.position.z
            tf_pose.transform.rotation.x = pose.orientation.x
            tf_pose.transform.rotation.y = pose.orientation.y
            tf_pose.transform.rotation.z = pose.orientation.z
            tf_pose.transform.rotation.w = pose.orientation.w

            self.static_tf_broadcaster.sendTransform(tf_pose)

    def publish_surfaces_and_base_positions(self):
        base_pos_idx = 0
        for block_name, block in self.blocks.items():
            while not self.tf_buffer.can_transform('world', block_name, Time()):
                pass

            for i, surface in enumerate(block.surfaces):
                tf_pose = TransformStamped()
                tf_pose.header.stamp = self.get_clock().now().to_msg()
                tf_pose.header.frame_id = block_name
                tf_pose.child_frame_id = f"{block_name}_surface{i}"
                tf_pose.transform.translation.x = surface.center[0]
                tf_pose.transform.translation.y = surface.center[1]
                tf_pose.transform.translation.z = surface.center[2]
                tf_pose.transform.rotation.x = 0.0
                tf_pose.transform.rotation.y = 0.0
                tf_pose.transform.rotation.z = 0.0
                tf_pose.transform.rotation.w = 1.0

                self.static_tf_broadcaster.sendTransform(tf_pose)
                self.get_logger().info(f"Published TF for {tf_pose.child_frame_id} at {surface.center} with normal {surface.normal}")

            base_positions = self.base_positions[base_pos_idx]
            for j, base_pos in enumerate(base_positions):
                base_pose_stamped = PoseStamped()
                base_pose_stamped.header.stamp = self.get_clock().now().to_msg()
                base_pose_stamped.header.frame_id = block_name
                base_pose_stamped.pose.position.x = base_pos[0]
                base_pose_stamped.pose.position.y = base_pos[1]
                base_pose_stamped.pose.position.z = base_pos[2]
                base_pose_stamped.pose.orientation.x = 0.0
                base_pose_stamped.pose.orientation.y = 0.0
                base_pose_stamped.pose.orientation.z = 0.0
                base_pose_stamped.pose.orientation.w = 1.0

                base_pose_in_world = self.tf_buffer.transform(base_pose_stamped, 'world', timeout=Duration(seconds=5.0))
                base_pose_in_world = cast(PoseStamped, base_pose_in_world)

                tf_pose = TransformStamped()
                tf_pose.header.stamp = self.get_clock().now().to_msg()
                tf_pose.header.frame_id = 'world'
                tf_pose.child_frame_id = f"{block_name}_base_target{j}"
                tf_pose.transform.translation.x = base_pose_in_world.pose.position.x
                tf_pose.transform.translation.y = base_pose_in_world.pose.position.y
                tf_pose.transform.translation.z = base_pose_in_world.pose.position.z
                tf_pose.transform.rotation.x = 0.0
                tf_pose.transform.rotation.y = 0.0
                tf_pose.transform.rotation.z = 0.0
                tf_pose.transform.rotation.w = 1.0

                self.static_tf_broadcaster.sendTransform(tf_pose)
                self.get_logger().info(f"Published TF for {tf_pose.child_frame_id} at {base_pos}")

            base_pos_idx += 1

    def gz_pose_callback(self, pose_v_msg):
        for pose_msg in pose_v_msg.pose:
            pose_name = pose_msg.name
            if pose_name in self.objs or pose_name == 'omnirob_iisy_vgc10':
                pose_name = pose_name if pose_name != 'omnirob_iisy_vgc10' else 'platform_base_link'
                tf_pose = TransformStamped()
                tf_pose.header.stamp = self.get_clock().now().to_msg()
                tf_pose.header.frame_id = 'world'
                tf_pose.child_frame_id = pose_name
                tf_pose.transform.translation.x = pose_msg.position.x
                tf_pose.transform.translation.y = pose_msg.position.y
                tf_pose.transform.translation.z = pose_msg.position.z
                tf_pose.transform.rotation.x = pose_msg.orientation.x
                tf_pose.transform.rotation.y = pose_msg.orientation.y
                tf_pose.transform.rotation.z = pose_msg.orientation.z
                tf_pose.transform.rotation.w = pose_msg.orientation.w

                self.tf_broadcaster.sendTransform(tf_pose)

                if pose_name == 'platform_base_link':
                    pose = Pose()
                    pose.position.x = pose_msg.position.x
                    pose.position.y = pose_msg.position.y
                    pose.position.z = pose_msg.position.z
                    pose.orientation.x = pose_msg.orientation.x
                    pose.orientation.y = pose_msg.orientation.y
                    pose.orientation.z = pose_msg.orientation.z
                    pose.orientation.w = pose_msg.orientation.w
                    self.robot_pose_pub.publish(pose)

    def request_plan(self) -> None:
        request = PlanConstructionTask.Request()

        request.config_name = self.problem_name
        # request.problem_config_path = self.config_path + '/'
        request.blocks = list(self.blocks.values())

        future = self.task_planner_client.call_async(request)
        future.add_done_callback(self.plan_response_callback)

    def plan_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Plan received successfully, executing plan...")
                self.send_execute_plan_request(response.plan)
            else:
                self.get_logger().error(f"Failed to receive plan: {response.msg}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def send_execute_plan_request(self, plan: Plan) -> None:
        request = ExecutePlan.Request()
        request.problem_name = self.problem_name
        request.plan = plan

        # Wait until tf tree is initialized
        while not self.tf_buffer.can_transform('world', 'platform_base_link', Time()):
            pass

        future = self.plan_executor_client.call_async(request)
        future.add_done_callback(self.execute_plan_response_callback)

    def execute_plan_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Plan executed successfully: {response.message}")
            else:
                self.get_logger().error(f"Failed to execute plan: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def find_pose_from_position(self, pos: Tuple[float, float, float]) -> Tuple[str, Pose]:
        for idx, position in enumerate(self.poses_list):
            if np.allclose(np.array(position), np.array(pos)):
                pose_name = list(self.poses_dict.keys())[idx]
                return pose_name, self.poses_dict[pose_name]

        pose_name = 'p' + str(len(self.poses_dict) + 1)
        pose = Pose()
        pose.position.x = pos[0]
        pose.position.y = pos[1]
        pose.position.z = (pos[2] + 1) * self.box_size / 2.0
        pose.orientation.w = 1.0

        return pose_name, pose

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