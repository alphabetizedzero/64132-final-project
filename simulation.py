from __future__ import print_function

import os
import sys
import argparse
import numpy as np

sys.path.extend(os.path.abspath(os.path.join(os.getcwd(), d)) for d in ['pddlstream', 'ss-pybullet'])

from pybullet_tools.utils import set_pose, Pose, Point, Euler, multiply, get_pose, get_point, create_box, set_all_static, WorldSaver, create_plane, COLOR_FROM_NAME, stable_z_on_aabb, pairwise_collision, elapsed_time, get_aabb_extent, get_aabb, create_cylinder, set_point, get_function_name, wait_for_user, dump_world, set_random_seed, set_numpy_seed, get_random_seed, get_numpy_seed, set_camera, set_camera_pose, link_from_name, get_movable_joints, get_joint_name
from pybullet_tools.utils import CIRCULAR_LIMITS, get_custom_limits, set_joint_positions, interval_generator, get_link_pose, interpolate_poses
# from pybullet_tools.utils import CIRCULAR_LIMITS, get_custom_limits, set_joint_positions, get_link_pose, interpolate_poses

from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO, FRANKA_URDF
from pybullet_tools.ikfast.ikfast import get_ik_joints, closest_inverse_kinematics

from src.world import World
from src.utils import JOINT_TEMPLATE, BLOCK_SIZES, BLOCK_COLORS, COUNTERS, \
    ALL_JOINTS, LEFT_CAMERA, CAMERA_MATRIX, CAMERA_POSES, CAMERAS, compute_surface_aabb, \
    BLOCK_TEMPLATE, name_from_type, GRASP_TYPES, SIDE_GRASP, joint_from_name, \
    STOVES, TOP_GRASP, randomize, LEFT_DOOR, point_from_pose, translate_linearly


import time
import bfs_planner
from robot_commands import *
from rrt_draft import *

UNIT_POSE2D = (0., 0., 0.)

def add_ycb(world, ycb_type, idx=0, counter=0, **kwargs):
    name = name_from_type(ycb_type, idx)
    world.add_body(name, color=np.ones(4))
    pose2d_on_surface(world, name, COUNTERS[counter], **kwargs)
    return name

def pose2d_on_surface(world, entity_name, surface_name, pose2d=UNIT_POSE2D):
    x, y, yaw = pose2d
    body = world.get_body(entity_name)
    surface_aabb = compute_surface_aabb(world, surface_name)
    z = stable_z_on_aabb(body, surface_aabb)
    pose = Pose(Point(x, y, z), Euler(yaw=yaw))
    set_pose(body, pose)
    return pose


add_sugar_box = lambda world, **kwargs: add_ycb(world, 'sugar_box', **kwargs)
add_spam_box = lambda world, **kwargs: add_ycb(world, 'potted_meat_can', **kwargs)


def execute_plan(world, locations, plan, action_impls):
    '''
    Execution engine responsible for executing the plan in the simulation environment

    @param world: the simulation environment
    @param locations: a dict(location_name (str): location_coords (1x3 np.array))
    @param plan: a list of tuples :(PDDLParser Action, dict(param_name (str) : param_value(str)))
    @param action_impls: a dictionary mapping action names (strings) to functions that implement them. The implementation
    functions should take arguments: world, locations, dict(param_name (str) : param_value(str))
    '''

    for i, (action, params) in enumerate(plan):

        if action.name not in action_impls:
            print(f'No implementation found for {action.name}. Skipping...')
            continue

        print(f'\t\t{i}\tExecuting action {action.name} with parameters {params}')
        action_impls[action.name](world, locations, params)


def main():   

    action_impls = {
        'move-robot-base': action_move_robot_base,
    }

    plan = bfs_planner.generate_plan('domain.pddl', 'problem.pddl')

    
    world = World(use_gui=True)
    world._update_initial()
    # TODO: test the locations
    sugar_box = add_sugar_box(world, idx=0, counter=1, pose2d=(-0.2, 0.65, np.pi / 4))
    spam_box = add_spam_box(world, idx=1, counter=0, pose2d=(0.2, 1.1, np.pi / 4))
    locations = {
        # Base locations should be (1x2) np.arrays (x, y)
        'robot-base-grasp-sugar': np.array([0.8 , 3.142]),  # Location of the base before grasping the sugar
        'robot-base-drop-sugar': np.array([0.8 , 3.142]),  # Location of the base before dropping the sugar
        'robot-base-grasp-spam': np.array([0.8 , 3.142]),  # Location of the base before grasping the spam
        'robot-base-drop-spam': np.array([0.8 , 3.142]),  # Location of the base before dropping the spam
        'robot-base-grasp-drawer': np.array([0.8 , 3.142]),  # Location of the base before grasping the drawer handle
        'robot-base-drop-drawer': np.array([1.1 , 3.142]),  # Location of the base before dropping the drawer handle

        # Arm locations should be Pose objects
        'robot-arm-home': get_link_pose(world.robot, tool_link),  # Pose of the arm when stowed
        'robot-arm-grasp-sugar': get_box_pose(world, sugar_box),  # Pose of the arm before grasping the sugar
        'robot-arm-drop-sugar': Pose(get_part_pose('kitchen_part_right', 'indigo_countertop')[0][:2] + [-0.25]),  # Pose of the arm before dropping the sugar
        'robot-arm-grasp-spam': get_box_pose(world, spam_box),  # Pose of the arm before grasping the spam
        'robot-arm-drop-spam': Pose(np.array(get_part_pose('kitchen_part_right', 'indigo_drawer_handle_top')[0][:2]+[-0.35])),  # Pose of the arm before dropping the spam
        'robot-arm-grasp-drawer': get_part_pose('kitchen_part_right', 'indigo_drawer_handle_top'),  # Pose of the arm before grasping the drawer, opening drawer can be handled by the robot base moving back? TODO discuss
    }
    print('Started')
    wait_for_user()
    execute_plan(world, locations, plan, action_impls)

    print('Done!')
    wait_for_user()


if __name__ == '__main__':
    main()
