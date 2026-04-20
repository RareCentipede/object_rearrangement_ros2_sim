import rclpy
import sys
import numpy as np

from typing import Tuple
from yaml import safe_load

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Pose, TransformStamped
from tf2_ros import TransformBroadcaster
from tf2_ros import StaticTransformBroadcaster

from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V #type: ignore
from gz.msgs10.entity_factory_pb2 import EntityFactory #type: ignore
from gz.msgs10.boolean_pb2 import Boolean #type: ignore

from mpnp_interfaces.msg import Plan
from mpnp_interfaces.srv import PlanConstructionTask

class WorldManager(Node):
    def __init__(self, problem_name: str):
        super().__init__('world_manager')
        self.gz_node = GzNode()

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

        self.define_objs_and_poses()
        self.publish_position_tfs()

        self.robot_pose_pub = self.create_publisher(Pose, '/omnirob_iisy_vgc10/pose', 10, callback_group=ReentrantCallbackGroup())
        self.gz_node.subscribe(Pose_V, '/world/empty/pose/info', self.gz_pose_callback)
        self.task_planner_client = self.create_client(PlanConstructionTask, 'plan_construction_task')
        while not self.task_planner_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for plan_construction_task service...')
        self.get_logger().info('plan_construction_task service online!')
        self.request_plan()

    def request_plan(self) -> None:
        request = PlanConstructionTask.Request()

        request.config_name = self.problem_name
        request.problem_config_path = self.config_path + '/'

        future = self.task_planner_client.call_async(request)
        future.add_done_callback(self.plan_response_callback)

    def plan_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Plan received successfully: {response.plan}")
            else:
                self.get_logger().error(f"Failed to receive plan: {response.msg}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

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

    def define_objs_and_poses(self):
        idx = 1

        for obj_name, info in self.init_config.items():
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

            self.spawn_object(obj_name, pose)

        for obj_name, info in self.goal_config.items():
            pos = info['position']
            pose_name, pose = self.find_pose_from_position(pos)

            if pose_name not in self.poses_dict:
                self.poses_list.append(pos)
                self.poses_dict[pose_name] = pose

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