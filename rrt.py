from __future__ import print_function

import os
import sys
import argparse
import numpy as np

sys.path.extend(os.path.abspath(os.path.join(os.getcwd(), d)) for d in ['pddlstream', 'ss-pybullet'])

from pybullet_tools.utils import set_pose, Pose, Point, Euler, multiply, get_pose, get_point, create_box, set_all_static, WorldSaver, create_plane, COLOR_FROM_NAME, stable_z_on_aabb, pairwise_collision, elapsed_time, get_aabb_extent, get_aabb, create_cylinder, set_point, get_function_name, wait_for_user, dump_world, set_random_seed, set_numpy_seed, get_random_seed, get_numpy_seed, set_camera, set_camera_pose, link_from_name, get_movable_joints, get_joint_name
from pybullet_tools.utils import CIRCULAR_LIMITS, get_custom_limits, set_joint_positions, interval_generator, get_link_pose, interpolate_poses, get_joint_positions
from pybullet_tools.utils import joints_from_names, get_distance, clone_body, remove_body, get_links, dump_body, get_bodies, body_from_name

from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO, FRANKA_URDF
from pybullet_tools.ikfast.ikfast import get_ik_joints, closest_inverse_kinematics

from src.world import World
from src.utils import JOINT_TEMPLATE, BLOCK_SIZES, BLOCK_COLORS, COUNTERS, \
    ALL_JOINTS, LEFT_CAMERA, CAMERA_MATRIX, CAMERA_POSES, CAMERAS, compute_surface_aabb, \
    BLOCK_TEMPLATE, name_from_type, GRASP_TYPES, SIDE_GRASP, joint_from_name, \
    STOVES, TOP_GRASP, randomize, LEFT_DOOR, point_from_pose, translate_linearly

UNIT_POSE2D = (0., 0., 0.)

def get_box_pose(world, box):
    '''
    Input: the world and an object id (string from dump_world())
    Output: Pose of the object
    '''
    return get_pose(world.get_body(box))

def get_part_pose(body_name, link_name):
    '''
    Input: an object id (string from dump_world()) and a part id (string from dump_world())
    Output: Pose of the part
    '''
    body = body_from_name(body_name)
    return get_link_pose(body, link_from_name(body, link_name))


def _pose_format_correct(pose):
    if not isinstance(pose, tuple): return False
    if not len(pose) == 2: return False
    if not all(isinstance(x, (tuple, list)) for x in pose): return False
    if not len(pose[0]) == 3: return False
    if not len(pose[1]) == 4: return False

    return True


def rrt(world, start_pose, start_joint_conf, end_pose):
    '''
    Input: the world (Enviroment), start_pose: Pose of the robot, end_pose: Pose of the target pose's gripper
    Output: list of Poses for the gripper from start_pose to end_pose
    '''

    assert _pose_format_correct(start_pose)
    assert _pose_format_correct(end_pose)

    assert isinstance(start_joint_conf, tuple)
    assert len(start_joint_conf) == 7

    start_joint_conf = tuple(start_joint_conf)

    def pose_to_key(pose, conf):

        assert _pose_format_correct(pose)

        position, rotation = pose
        return (tuple(position), tuple(rotation)), tuple(conf)

    def collision_check(start_pose, end_pose):

        assert _pose_format_correct(start_pose)
        assert _pose_format_correct(end_pose)

        for pose in interpolate_poses(start_pose, end_pose, pos_step_size=0.2):
            conf = next(closest_inverse_kinematics(world.robot, PANDA_INFO, tool_link, pose, max_time=0.05), None)
            if conf is None:
                return True
            set_joint_positions(sub_robot, ik_joints, conf)
            for obs in world.static_obstacles:
                if pairwise_collision(sub_robot, obs):
                    print('Collide with', obs)
                    return True
        return False

    def distance(pos1, pos2):
        return get_distance(pos1, pos2)

    def nearest(verts, pos):
        min_dis = float('inf')
        closest = None
        for pose, joint_confs in verts:
            dis = distance(pose[0], point_from_pose(pos))
            if dis < min_dis:
                min_dis = dis
                closest = pose, joint_confs
        return closest

    def path_maker(edges, end_node, end_conf, start_node):

        assert _pose_format_correct(end_pose)

        current_node = pose_to_key(end_node, end_conf)
        path_list = [current_node]

        start_key = pose_to_key(start_node, start_joint_conf)

        while current_node != start_key:
            parent_pose, parent_joint_confs = edges[current_node]

            assert _pose_format_correct(parent_pose)

            parent_key = pose_to_key(parent_pose, parent_joint_confs)

            path_list.append(parent_key)

            current_node = parent_key

        # path_list.append(start_key)
        path_list.reverse()
        return path_list

    def steer(start_pose, end_pose, distance):
        pose = interpolate_poses(start_pose, end_pose, pos_step_size=distance, ori_step_size=np.pi/4)
        next(pose)
        return next(pose)

    def get_sample():

        # Normal point sampling
        sample_conf = sample_fn()
        set_joint_positions(sub_robot, sub_arm_joints, sample_conf)
        sample_x_random = get_link_pose(sub_robot, tool_link)

        goal_sample = np.random.uniform()
        if goal_sample < .8:
            return sample_x_random, sample_conf

        x_random = end_pose
        # print(f'Sampling goal')

        # Calculate conf for end pose
        for pose in interpolate_poses(start_pose, x_random, pos_step_size=0.01):
            conf = next(closest_inverse_kinematics(world.robot, PANDA_INFO, tool_link, pose, max_time=0.05), None)
            if conf is None:  # If the IK fails, revert to normal point sampling
                # print('rrt failure: no IK solution found while sampling goal')
                return sample_x_random, sample_conf

        return x_random, conf

    assert _pose_format_correct(start_pose)
    V = {pose_to_key(start_pose, start_joint_conf)}  # Each node in the graph is a tuple of (pose_key, joint_configuration tuple)
    E = dict()

    sub_robot = clone_body(world.robot, visual=False, collision=True)
    # sub_robot = world.robot
    sub_arm_joints = world.arm_joints
    sample_fn = get_sample_fn(sub_robot, sub_arm_joints)
    tool_link = link_from_name(world.robot, 'panda_hand')
    ik_joints = get_ik_joints(world.robot, PANDA_INFO, tool_link)


    for _ in range(100000):

        x_random, conf = get_sample()  # Sample the joint space

        if pose_to_key(x_random, conf) not in E.keys():

            nearest_node_pose, nearest_node_joint_confs = nearest(V, x_random)
            # x_new = steer(nearest_node, x_random, 0.1)
            x_new = x_random

            if not collision_check(nearest_node_pose, x_new):

                new_V = pose_to_key(x_new, conf)
                V.add(new_V)
                E[new_V] = pose_to_key(nearest_node_pose, nearest_node_joint_confs)

                if distance(point_from_pose(end_pose), point_from_pose(x_new)) < 0.01:
                    sol = path_maker(E, x_new, conf, start_pose)
                    remove_body(sub_robot)
                    return sol

    remove_body(sub_robot)

    return None


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

def get_sample_fn(body, joints, custom_limits={}, **kwargs):
    lower_limits, upper_limits = get_custom_limits(body, joints, custom_limits, circular_limits=CIRCULAR_LIMITS)
    generator = interval_generator(lower_limits, upper_limits, **kwargs)
    def fn():
        return tuple(next(generator))
    return fn

def main():
    print('Random seed:', get_random_seed())
    print('Numpy seed:', get_numpy_seed())

    np.set_printoptions(precision=3, suppress=True)
    world = World(use_gui=True)
    sugar_box = add_sugar_box(world, idx=0, counter=1, pose2d=(-0.2, 0.65, np.pi / 4))
    spam_box = add_spam_box(world, idx=1, counter=0, pose2d=(0.2, 1.1, np.pi / 4))
    world._update_initial()

    # dump_world()
    # print("sugar_box pose =", get_box_pose(world, sugar_box))
    # print("spam_box pose =", get_box_pose(world, spam_box))
    # print("pose of indigo_drawer_handle_top at kitchen_part_right =", get_part_pose('kitchen_part_right', 'indigo_drawer_handle_top'))

    goal_pos = translate_linearly(world, 1.2)  # does not do any collision checking!!
    set_joint_positions(world.robot, world.base_joints, goal_pos)
    tool_link = link_from_name(world.robot, 'panda_hand')

    ik_joints = get_ik_joints(world.robot, PANDA_INFO, tool_link)
    start_joint_conf = get_joint_positions(world.robot, ik_joints)

    sample_fn = get_sample_fn(world.robot, world.arm_joints)
    conf_start = sample_fn()
    set_joint_positions(world.robot, world.arm_joints, conf_start)
    start_pose = get_link_pose(world.robot, tool_link)

    conf_end = sample_fn()
    set_joint_positions(world.robot, world.arm_joints, conf_end)
    end_pose = get_link_pose(world.robot, tool_link)
    set_joint_positions(world.robot, world.arm_joints, conf_start)

    wait_for_user()
    paths = rrt(world, start_pose, start_joint_conf, end_pose)

    print(f"Start Pose: {start_pose}")
    print(f'End Pose: {end_pose}')

    print(f'\n\nFound path with {len(paths)} steps:')
    for i, (pose, joint_conf) in enumerate(paths):
        print(f'Step {i}\n\tpose: {pose}\n\tjoint_conf: {joint_conf}')
    print('\n\n')

    print("Found solution")
    wait_for_user()
    sp, sp_joint_confs = paths[0]
    for pose, joint_conf in paths[1:]:
        print(f'Moving to pose: {pose}')

        # IK for smooth motion
        for interp_pose in interpolate_poses(sp, pose, pos_step_size=0.01):
            conf = next(closest_inverse_kinematics(world.robot, PANDA_INFO, tool_link, interp_pose, max_time=0.05), None)
            if conf is None:
                print('IK failure!')
                wait_for_user()
                break
            set_joint_positions(world.robot, ik_joints, conf)

        set_joint_positions(world.robot, ik_joints, joint_conf)
        sp = pose
        wait_for_user()
    print('End Pose')
    wait_for_user()
    world.destroy()

if __name__ == '__main__':
    main()
