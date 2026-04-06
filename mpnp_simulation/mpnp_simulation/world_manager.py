import rclpy
import yaml
import sys
import numpy as np

from typing import List, Tuple

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
from tf2_ros import StaticTransformBroadcaster

from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V #type: ignore

class WorldManager(Node):
    def __init__(self, problem_name: str):
        super().__init__('world_manager')

        self.config_path = 'src/object_rearrangement_ros2_sim/mpnp_simulation/config/problem_configs'
        self.problem_path = f'{self.config_path}/{problem_name}'
        self.init_config = yaml.safe_load(open(f'{self.problem_path}/init.yaml', 'r'))
        self.goal_config = yaml.safe_load(open(f'{self.problem_path}/goal.yaml', 'r'))

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.poses_dict = {}
        self.poses_list = []
        self.objs = []

        self.get_logger().info(f'Loaded problem: {problem_name}')

        self.define_objs_and_poses()
        self.publish_position_tfs()

        self.gz_node = GzNode()
        self.gz_node.subscribe(Pose_V, '/world/empty/dynamic_pose/info', self.gz_pose_callback)

    def gz_pose_callback(self, pose_v_msg):
        for pose_msg in pose_v_msg.pose:
            pose_name = pose_msg.name
            if pose_name in self.objs:
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

    def define_objs_and_poses(self):
        idx = 1

        for obj_name, info in self.init_config.items():
            self.objs.append(obj_name)
            pose = Pose()

            pose.position.x = info['position'][0]
            pose.position.y = info['position'][1]
            pose.position.z = 0.15
            pose.orientation.x = info['orientation'][0]
            pose.orientation.y = info['orientation'][1]
            pose.orientation.z = info['orientation'][2]
            pose.orientation.w = 1.0

            self.poses_list.append([pose.position.x, pose.position.y, pose.position.z])

            pose_name = 'p' + str(idx)
            self.poses_dict[pose_name] = pose
            idx += 1

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
        pose.position.z = 0.15
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