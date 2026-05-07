import rclpy
import sys
import numpy as np

from typing import Tuple, cast, List
from yaml import safe_load
from scipy.spatial.transform import Rotation as R

from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Transform, TransformStamped, Point, Quaternion, Vector3, PoseStamped
from tf2_geometry_msgs import PoseStamped as TF2PoseStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster, Buffer, TransformListener

from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V #type: ignore
from gz.msgs10.entity_factory_pb2 import EntityFactory #type: ignore
from gz.msgs10.boolean_pb2 import Boolean #type: ignore

from mpnp_interfaces.msg import Plan, Surface, Block
from mpnp_interfaces.srv import PlanConstructionTask, ExecutePlan

from mpnp_simulation.scripts.generate_polyhedrals import generate_diced_block, compute_base_positions

class WorldManager(Node):
    def __init__(self, problem_name: str, execute_plan: bool = True):
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

        self.execute = execute_plan

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.poses_dict = {}
        self.poses_list = []
        self.objs = []
        self.blocks = {}
        self.robot_init_pose = PoseStamped()

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

        robot_init_data = self.init_config.pop('robot')
        self.robot_init_pose = PoseStamped(
            header=Header(
                stamp=self.get_clock().now().to_msg(),
                frame_id='p' + str(len(self.init_config)+1)
            ),
            pose=Pose(
                position=Point(
                    x=robot_init_data['position'][0],
                    y=robot_init_data['position'][1],
                    z=robot_init_data['position'][2]
                ),
                orientation=Quaternion(
                    x=robot_init_data['orientation'][0],
                    y=robot_init_data['orientation'][1],
                    z=robot_init_data['orientation'][2],
                )
            )
        )
        self.poses_dict[self.robot_init_pose.header.frame_id] = self.robot_init_pose

        for obj_name, info in self.init_config.items():
            self.objs.append(obj_name)
            pose_name = 'p' + str(idx)
            pose_stamped = PoseStamped(
                header=Header(
                    stamp=self.get_clock().now().to_msg(),
                    frame_id=pose_name
                ),
                pose=Pose(
                    position=Point(
                        x=info['position'][0],
                        y=info['position'][1],
                        z=(info['position'][2] + 1) * self.box_size / 2.0
                    ),
                    orientation=Quaternion(
                        x=info['orientation'][0],
                        y=info['orientation'][1],
                        z=info['orientation'][2],
                    )
                )
            )

            self.poses_list.append([pose_stamped.pose.position.x, pose_stamped.pose.position.y, pose_stamped.pose.position.z])

            self.poses_dict[pose_name] = pose_stamped
            idx += 1

            block = Block()
            block.name = obj_name
            block.init_pose = pose_stamped
            self.blocks[obj_name] = block
            self.spawn_object(obj_name, pose_stamped)

        for obj_name, info in self.goal_config.items():
            pos = info['position']
            pose_name, pose_stamped = self.find_pose_from_position(pos)

            if pose_name not in self.poses_dict:
                self.poses_list.append(pos)

                orientation = info.get('orientation', [0.0, 0.0, 0.0])
                pose_stamped.pose.orientation = Quaternion(x=orientation[0], y=orientation[1], z=orientation[2])
                self.poses_dict[pose_name] = pose_stamped

                block = self.blocks[obj_name]
                block.goal_pose = pose_stamped
                self.blocks.update({obj_name: block})

    def spawn_object(self, obj_name: str, pose: PoseStamped):
        spawn_obj_req = EntityFactory()
        spawn_obj_req.sdf = open(f'src/object_rearrangement_ros2_sim/mpnp_simulation/models/box.sdf', 'r').read()
        spawn_obj_req.pose.position.x = pose.pose.position.x
        spawn_obj_req.pose.position.y = pose.pose.position.y
        spawn_obj_req.pose.position.z = pose.pose.position.z
        spawn_obj_req.pose.orientation.x = pose.pose.orientation.x
        spawn_obj_req.pose.orientation.y = pose.pose.orientation.y
        spawn_obj_req.pose.orientation.z = pose.pose.orientation.z
        spawn_obj_req.pose.orientation.w = pose.pose.orientation.w
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
            base_positions = []

            _, _, clean_faces = generate_diced_block(self.box_size/2, 14)
            base_placements, valid_centers, valid_normals = compute_base_positions(clean_faces)

            block = self.blocks[obj_name]
            for c, n in zip(valid_centers, valid_normals):
                surfaces.append(Surface(
                    center=Vector3(
                        x=c[0],
                        y=c[1],
                        z=c[2]
                    ),
                    normal=Vector3(
                        x=n[0],
                        y=n[1],
                        z=n[2]
                    )
                ))
            block.surfaces = surfaces

            for base_pos in base_placements:
                base_pose = PoseStamped(
                    header=Header(
                            frame_id=obj_name
                        ),
                    pose=Pose(
                        position=Point(
                            x=base_pos[0],
                            y=base_pos[1],
                            z=base_pos[2]
                        ),
                        orientation=block.init_pose.pose.orientation
                    )
                )
                base_positions.append(base_pose)

            block.base_positions = base_positions
            self.blocks.update({obj_name: block})

    def publish_position_tfs(self):
        for pose_name, pose_stamped in self.poses_dict.items():
            tf_pose = TransformStamped(
                header=Header(
                    stamp=self.get_clock().now().to_msg(),
                    frame_id='world'
                ),
                child_frame_id=pose_name,
                transform=Transform(
                    translation=Vector3(
                        x=pose_stamped.pose.position.x,
                        y=pose_stamped.pose.position.y,
                        z=pose_stamped.pose.position.z
                    ),
                    rotation=Quaternion(
                        x=pose_stamped.pose.orientation.x,
                        y=pose_stamped.pose.orientation.y,
                        z=pose_stamped.pose.orientation.z,
                        w=pose_stamped.pose.orientation.w
                    )
                )
            )

            self.static_tf_broadcaster.sendTransform(tf_pose)

    def publish_surfaces_and_base_positions(self):
        for block_name, block in self.blocks.items():
            while not self.tf_buffer.can_transform('world', block_name, Time()):
                self.get_logger().info(f'Waiting for transform from {block_name} to world to publish surfaces and base positions...')
                self.create_rate(1.0).sleep()

            for i, surface in enumerate(block.surfaces):
                surface_normal_quat = normal_to_quaternion([surface.normal.x, surface.normal.y, surface.normal.z])
                tf_pose = TransformStamped(
                    header=Header(
                        stamp=self.get_clock().now().to_msg(),
                        frame_id=block_name
                    ),
                    child_frame_id=f"{block_name}_surface{i}",
                    transform=Transform(
                        translation=surface.center,
                        rotation=surface_normal_quat
                    )
                )

                self.static_tf_broadcaster.sendTransform(tf_pose)

            pick_poses_stamped, place_poses_stamped = self.publish_base_positions(block)
            block.base_positions = pick_poses_stamped
            block.place_positions = place_poses_stamped
            self.blocks.update({block_name: block})

    def publish_base_positions(self, block: Block):
        pick_poses_stamped = []
        place_poses_stamped = []
        goal_pose_stamped = block.goal_pose
        goal_pose_name = goal_pose_stamped.header.frame_id if goal_pose_stamped else ''
        for j, base_pose_stamped in enumerate(block.base_positions):
            base_pose_stamped.header.stamp = self.get_clock().now().to_msg()

            base_pose_in_world = self.tf_buffer.transform(
                base_pose_stamped, 'world', timeout=Duration(seconds=5.0)
            )
            base_pose_in_world = cast(PoseStamped, base_pose_in_world)
            base_pose_in_world.header.frame_id = f"{block.name}_base_target{j}"
            base_pose_in_world.pose.orientation = rotate_pose_to_target(base_pose_in_world, block.init_pose)

            tf_pose = TransformStamped(
                header=Header(
                    stamp=self.get_clock().now().to_msg(),
                    frame_id='world'
                ),
                child_frame_id=f"{block.name}_base_target{j}",
                transform=Transform(
                    translation=Vector3(
                        x=base_pose_in_world.pose.position.x,
                        y=base_pose_in_world.pose.position.y,
                        z=base_pose_in_world.pose.position.z
                    ),
                    rotation=base_pose_in_world.pose.orientation
                )
            )

            self.static_tf_broadcaster.sendTransform(tf_pose)
            pick_poses_stamped.append(base_pose_in_world)
            self.poses_dict[base_pose_in_world.header.frame_id] = base_pose_in_world

            if goal_pose_name != '':
                base_pose_stamped.header.frame_id = goal_pose_name
                place_pose_in_world = self.tf_buffer.transform(
                    base_pose_stamped, 'world', timeout=Duration(seconds=5.0)
                )

                place_pose_in_world = cast(PoseStamped, place_pose_in_world)
                place_pose_in_world.header.frame_id = f"{block.name}_place_target{j}"
                place_pose_in_world.pose.orientation = rotate_pose_to_target(place_pose_in_world, goal_pose_stamped)

                place_pose_in_world_tf = TransformStamped(
                    header=Header(
                        stamp=self.get_clock().now().to_msg(),
                        frame_id='world'
                    ),
                    child_frame_id=f"{block.name}_place_target{j}",
                    transform=Transform(
                        translation=Vector3(
                            x=place_pose_in_world.pose.position.x,
                            y=place_pose_in_world.pose.position.y,
                            z=place_pose_in_world.pose.position.z
                        ),
                        rotation=place_pose_in_world.pose.orientation
                    )
                )

                self.static_tf_broadcaster.sendTransform(place_pose_in_world_tf)
                place_poses_stamped.append(place_pose_in_world)
                self.poses_dict[place_pose_in_world.header.frame_id] = place_pose_in_world

        return pick_poses_stamped, place_poses_stamped

    def gz_pose_callback(self, pose_v_msg):
        for pose_msg in pose_v_msg.pose:
            pose_name = pose_msg.name
            if pose_name in self.objs or pose_name == 'omnirob_iisy_vgc10':
                pose_name = pose_name if pose_name != 'omnirob_iisy_vgc10' else 'platform_base_link'
                tf_pose = TransformStamped(
                    header=Header(
                        stamp=self.get_clock().now().to_msg(),
                        frame_id='world'
                    ),
                    child_frame_id=pose_name,
                    transform=Transform(
                        translation=Vector3(
                            x=pose_msg.position.x,
                            y=pose_msg.position.y,
                            z=pose_msg.position.z
                        ),
                        rotation=Quaternion(
                            x=pose_msg.orientation.x,
                            y=pose_msg.orientation.y,
                            z=pose_msg.orientation.z,
                            w=pose_msg.orientation.w
                        )
                    )
                )

                self.tf_broadcaster.sendTransform(tf_pose)
                self.get_logger().debug(f"Published TF for {pose_name} at position "
                                        f"({pose_msg.position.x}, {pose_msg.position.y}, {pose_msg.position.z})")

                if pose_name == 'platform_base_link':
                    pose = Pose(
                        position=Point(
                            x=pose_msg.position.x,
                            y=pose_msg.position.y,
                            z=pose_msg.position.z
                        ),
                        orientation=Quaternion(
                            x=pose_msg.orientation.x,
                            y=pose_msg.orientation.y,
                            z=pose_msg.orientation.z,
                            w=pose_msg.orientation.w
                        )
                    )
                    self.robot_pose_pub.publish(pose)

    def request_plan(self) -> None:
        request = PlanConstructionTask.Request()

        request.config_name = self.problem_name
        # request.problem_config_path = self.config_path + '/'
        request.blocks = list(self.blocks.values())
        request.robot_init_pose = self.robot_init_pose
        # self.get_logger().info(f"Requesting plan with blocks: {[block.name for block in request.blocks]}")

        future = self.task_planner_client.call_async(request)
        future.add_done_callback(self.plan_response_callback)

    def plan_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Plan received successfully, executing plan...")
                if self.execute:
                    self.send_execute_plan_request(response.plan)
                else:
                    self.get_logger().info(f"Execution disabled, not sending execute plan request.")
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

    def find_pose_from_position(self, pos: Tuple[float, float, float]) -> Tuple[str, PoseStamped]:
        for idx, position in enumerate(self.poses_list):
            if np.allclose(np.array(position), np.array(pos)):
                pose_name = list(self.poses_dict.keys())[idx]
                return pose_name, self.poses_dict[pose_name]

        pose_name = 'p' + str(len(self.poses_dict) + 1)
        pose = PoseStamped(
            header=Header(
                stamp=self.get_clock().now().to_msg(),
                frame_id=pose_name
            ),
            pose=Pose(
                position=Point(
                    x=pos[0],
                    y=pos[1],
                    z=(pos[2] + 1) * self.box_size / 2.0
                )
            )
        )

        return pose_name, pose

def normal_to_quaternion(normal) -> Quaternion:
    normal = normal / np.linalg.norm(normal)
    z = np.array([0, 0, 1])
    axis = np.cross(z, normal)
    angle = np.arccos(np.dot(z, normal))

    if angle < 1e-6:
        return Quaternion(x=0, y=0, z=0, w=1)

    quat = R.from_rotvec(axis / np.linalg.norm(axis) * angle).as_quat(False)
    quat_msg = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])

    return quat_msg

def rotate_pose_to_target(current_pose: PoseStamped, target_pose: PoseStamped) -> Quaternion:
    pos = [current_pose.pose.position.x, current_pose.pose.position.y]
    target_pos = [target_pose.pose.position.x, target_pose.pose.position.y]

    pos_to_target_vec = np.array(target_pos) - np.array(pos)
    target_angle = np.arctan2(pos_to_target_vec[1], pos_to_target_vec[0])
    quat = R.from_euler('z', target_angle).as_quat(False)
    quat_msg = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])

    return quat_msg

def main():
    args = sys.argv[1:]
    rclpy.init()
    problem_name = args[0] if args else 'basic'
    execute = False if args[1] == 'false' and args and len(args) > 1 else True
    world_manager = WorldManager(problem_name, execute)
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