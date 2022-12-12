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
from rrt import *

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

    print('Random seed:', get_random_seed())
    print('Numpy seed:', get_numpy_seed())

    action_impls = {
        'move-robot-base': action_move_robot_base,
        'pick-up-sugar': action_pick_up_sugar,
    }

    plan = bfs_planner.generate_plan('domain.pddl', 'problem.pddl')
    print(f'Generated plan with {len(plan)} steps')



    world = World(use_gui=True)
    world._update_initial()

    sugar_box = add_sugar_box(world, idx=0, counter=1, pose2d=(-0.2, 0.65, np.pi / 4))
    spam_box = add_spam_box(world, idx=1, counter=0, pose2d=(0.2, 1.1, np.pi / 4))


    locations = {
        # Base locations should be (1x2) np.arrays (x, y)
        'robot-base-grasp-sugar': np.array([0.7, 0.65, np.pi / 2]),  # Location of the base before grasping the sugar
        'robot-base-drop-sugar': np.array([0.7, 0.65, np.pi / 2]),  # Location of the base before dropping the sugar
        'robot-base-grasp-spam': np.array([0.7, 0.65, np.pi / 2]),  # Location of the base before grasping the spam
        'robot-base-drop-spam': np.array([0.7, 0.65, np.pi / 2]),  # Location of the base before dropping the spam
        'robot-base-open-drawer': np.array([0.7, 0.65, np.pi / 2]),  # Location of the base before grasping the drawer handle

        # Arm locations should be Pose objects
        'robot-arm-home': get_link_pose(world.robot, link_from_name(world.robot, 'panda_hand')),  # Pose of the arm when stowed
        'robot-arm-grasp-sugar': Pose([-0.109, 0.741, -0.444], [0, -np.pi/2, np.pi/4]),  # Pose of the arm before grasping the sugar
        'robot-arm-drop-sugar': Pose([0.051, 1.318, -0.444], [0, -np.pi / 2, 0]),  # Pose of the arm before dropping the sugar
        'robot-arm-grasp-spam': Pose([0.192, 1.058, -0.341], [np.pi, 0, np.pi/4]),  # Pose of the arm before grasping the spam
        'robot-arm-drop-spam': Pose([0.501, 1.211, -0.544], [np.pi, 0, 0]),  # Pose of the arm before dropping the spam
        'robot-arm-open-drawer': Pose([0.428, 1.211, -0.654], [np.pi/2, 0, -np.pi/2]),  # Pose of the arm before grasping the drawer, opening drawer can be handled by the robot base moving back? TODO discuss
        'robot-arm-opened-drawer': Pose([0.628, 1.211, -0.654], [np.pi/2, 0, -np.pi/2]),  # Pose of the arm before grasping the drawer, opening drawer can be handled by the robot base moving back? TODO discuss
    }


    execute_plan(world, locations, plan, action_impls)









def examples():

    ######################################################################
    # Example (working): move arm small amount far from objects
    ######################################################################
    # move_robot_base(world, np.array([2, 2]))
    # rotate_robot_base(world, target_heading)
    # # (0.34579718112945557, 0.6452866196632385, -0.24738281965255737), (-0.026103319600224495, 0.9215595126152039, 0.00632764957845211, -0.38730689883232117)
    # target_arm_pose = ((1.645, 1.995, -0.147), (-0.026103319600224495, 0.9215595126152039, 0.00632764957845211, -0.38730689883232117))
    # move_robot_arm(world, target_arm_pose)
    
    # print(f'Final arm pose: {get_link_pose(world.robot, tool_link)}')
    
    # wait_for_user()
    ######################################################################
    ######################################################################


    ######################################################################
    # Example (working): move arm large amount far from obstacles
    ######################################################################
    # move_robot_base(world, np.array([2, 2, np.pi]))
    # rotate_robot_base(world, target_heading)
    # # (0.34579718112945557, 0.6452866196632385, -0.24738281965255737), (-0.026103319600224495, 0.9215595126152039, 0.00632764957845211, -0.38730689883232117)
    # target_arm_pose = ((1.445, 2.295, -0.047), (-0.026103319600224495, 0.9215595126152039, 0.00632764957845211, -0.38730689883232117))
    # move_robot_arm(world, target_arm_pose)

    # print(f'Final arm pose: {get_link_pose(world.robot, tool_link)}')

    # wait_for_user()
    ######################################################################
    ######################################################################


    ######################################################################
    # Example (working): move arm small amount near obstacles
    ######################################################################
    # move_robot_base(world, np.array([0.7, 0.65, np.pi/2]))
    # # (0.34579718112945557, 0.6452866196632385, -0.24738281965255737), (-0.026103319600224495, 0.9215595126152039, 0.00632764957845211, -0.38730689883232117)
    
    # print(f'Arm pose after base move: {get_link_pose(world.robot, tool_link)}')
    # # (0.34579718112945557, 0.6452866196632385, -0.24738281965255737), (-0.026103319600224495, 0.9215595126152039, 0.00632764957845211, -0.38730689883232117)

    # target_arm_pose = ((0.295, 0.645, -0.247), (-0.026103319600224495, 0.9215595126152039, 0.00632764957845211, -0.38730689883232117))
    # move_robot_arm(world, target_arm_pose)

    # print(f'Final arm pose: {get_link_pose(world.robot, tool_link)}')

    # wait_for_user()
    ######################################################################
    ######################################################################

    pass



if __name__ == '__main__':
    main()
