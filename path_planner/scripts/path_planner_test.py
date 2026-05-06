import numpy as np
import matplotlib.pyplot as plt

from yaml import safe_load
from mpnp_interfaces.msg import Block
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Point
from mapping import OCCUPANCY

from astar import GridGraph

def main():
    blocks = []
    problem_config_path = "src/object_rearrangement_ros2_sim/mpnp_simulation/config/problem_configs/"
    problem_name = "basic"

    init_config_path = problem_config_path + problem_name + "/init.yaml"
    goal_config_path = problem_config_path + problem_name + "/goal.yaml"

    init_config = safe_load(open(init_config_path, 'r'))
    goal_config = safe_load(open(goal_config_path, 'r'))

    for obj_name, obj_info in init_config.items():
        block = Block()
        block.name = obj_name
        goal_info = goal_config.get(obj_name, None)
        if goal_info:
            goal_pose = goal_info['position']
        else:
            goal_pose = None

        block.init_pose = PoseStamped(
            header=Header(
                frame_id="world",
            ),
            pose=Pose(
                position=Point(
                    x=obj_info['position'][0],
                    y=obj_info['position'][1],
                    z=obj_info['position'][2]
                )
            )
        )

        if not goal_pose:
            blocks.append(block)
            continue

        block.goal_pose = PoseStamped(
            header=Header(
                frame_id="world",
            ),
            pose=Pose(
                position=Point(
                    x=goal_pose[0],
                    y=goal_pose[1],
                    z=goal_pose[2]
                )
            )
        )
        blocks.append(block)

    gg = GridGraph(blocks, block_size=0.3, margin=0.2, connectivity=8)
    start_world = [0.0, 0.0]
    goal_world = [-4.0, 2.5]

    path = gg.plan(start_world, goal_world)
    gg.plot_grid_and_path(path, start_world, goal_world)

    # Move block to somewhere else and replan
    if len(blocks) > 0:
        block_to_move = blocks[0]
        old_pos = [block_to_move.init_pose.pose.position.x, block_to_move.init_pose.pose.position.y]
        new_pos = [block_to_move.init_pose.pose.position.x + 2, block_to_move.init_pose.pose.position.y + 2]
        gg.update_block_move(old_pos, OCCUPANCY.FREE)
        gg.update_block_move(new_pos, OCCUPANCY.OCCUPIED)

        new_path = gg.plan(start_world, goal_world)
        gg.plot_grid_and_path(new_path, start_world, goal_world)

    plt.show()

if __name__ == "__main__":
    main()