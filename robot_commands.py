import numpy as np

from pybullet_tools.utils import set_pose, Pose, Point, Euler, multiply, get_pose, get_point, create_box, set_all_static, WorldSaver, create_plane, COLOR_FROM_NAME, stable_z_on_aabb, pairwise_collision, elapsed_time, get_aabb_extent, get_aabb, create_cylinder, set_point, get_function_name, wait_for_user, dump_world, set_random_seed, set_numpy_seed, get_random_seed, get_numpy_seed, set_camera, set_camera_pose, link_from_name, get_movable_joints, get_joint_name
from pybullet_tools.utils import CIRCULAR_LIMITS, get_custom_limits, get_joint_positions, set_joint_positions, interval_generator, get_link_pose, interpolate_poses

from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO, FRANKA_URDF
from pybullet_tools.ikfast.ikfast import get_ik_joints, closest_inverse_kinematics

import time

_TIME_STEP = 0.1  # Update period in seconds

def rotate_robot_base(world, target_heading, angular_speed=0.5):
    '''
    Blocking, rotates the robot base to be pointing at the target heading

    @param world: world object
    @param target_heading: the target heading in radians
    @param angular_speed: the target angular speed in rad/sec
    '''

    x, y, current_heading = get_joint_positions(world.robot, world.base_joints)

    step_size = angular_speed * _TIME_STEP  # step size in rad/step
    direction = np.sign(target_heading - current_heading)

    while abs(current_heading - target_heading) > step_size:
        current_heading += direction * step_size

        pos = np.array([x, y, current_heading])

        set_joint_positions(world.robot, world.base_joints, pos)
        time.sleep(_TIME_STEP)

    set_joint_positions(world.robot, world.base_joints, np.array([x, y, target_heading]))


def move_robot_base(world, target_position, translation_v=1, angular_v=1):
    '''
    Blocking, moves the robot base to the target position

    @param world: world object
    @param target_position: the target position of the robot base as a (1x3) np.array
    @param translation_v: the translational velocity to command in m/sec
    @param angular_v: the angular velocity to command in rad/sec
    '''

    x, y, _ = get_joint_positions(world.robot, world.base_joints)

    target_position = np.reshape(target_position, (3,))
    target_x, target_y = target_position[0:2]

    target_heading = np.arctan((target_y - y) / (target_x - x))

    rotate_robot_base(world, target_heading, angular_v)  # First rotate the robot to be pointing the right way so the motion looks correct

    current_position = np.array([x, y])
    target_position = target_position[0:2]

    step_size = translation_v * _TIME_STEP
    direction = (target_position - current_position) / np.linalg.norm(target_position - current_position)  # unit direction vector

    # Take small intermediate steps to make the motion appear smooth rather than teleporting
    # right to the target position
    while np.all(abs(target_position - current_position) > step_size):
        current_position += direction * step_size

        set_joint_positions(world.robot, world.base_joints, np.append(current_position, [target_heading]))
        time.sleep(_TIME_STEP)

    set_joint_positions(world.robot, world.base_joints, np.append(target_position, [target_heading]))


def _get_sample_fn(body, joints, custom_limits={}, **kwargs):
    '''
    Copied from minimal_example.py
    '''
    lower_limits, upper_limits = get_custom_limits(body, joints, custom_limits, circular_limits=CIRCULAR_LIMITS)
    generator = interval_generator(lower_limits, upper_limits, **kwargs)
    def fn():
        return tuple(next(generator))
    return fn


def move_robot_arm(world, target_pose):
    '''
    Blocking, moves the robot end effector to the target_pose

    @param world: world object
    @param target_pose: Pose object that the end effector will move to
    '''

    tool_link = link_from_name(world.robot, 'panda_hand')

    sample_fn = _get_sample_fn(world.robot, world.arm_joints)

    conf = sample_fn()
    set_joint_positions(world.robot, world.arm_joints, conf)
    ik_joints = get_ik_joints(world.robot, PANDA_INFO, tool_link)
    start_pose = get_link_pose(world.robot, tool_link)
    
    for pose in interpolate_poses(start_pose, target_pose, pos_step_size=0.01):
        conf = next(closest_inverse_kinematics(world.robot, PANDA_INFO, tool_link, pose, max_time=0.05), None)
        if conf is None:
            print('Failure!')
            wait_for_user()
            break
        set_joint_positions(world.robot, ik_joints, conf)


# TODO: add grasp command




################################################################
#
#   PDDL Action Implementations
#
################################################################

_HOME_HEADING = np.pi  # Heading to return to after moving the base

def action_move_robot_base(world, locations, params):
    '''
    Performs the move-robot-base action

    @param world: the simulation environment
    @param locations: a dict(location_name (str): location_coords (1x3 np.array))
    @param params: dict(param_name (str) : param_value(str)). Should have param_names '?from-zone' and '?to-zone'
    '''
    
    assert all(param_name in params.keys() for param_name in {'?from-zone', '?to-zone'}), f'Incorrect params {params}'
    assert params['?to-zone'] in locations, f"Unknown location {params['?to-zone']}"

    target_position = locations[params['?to-zone']]

    move_robot_base(world, target_position)
    rotate_robot_base(world, _HOME_HEADING)


def action_pick_up_sugar(world, locations, params):
    '''
    Performs the pick-up-sugar action

    @param world: the simulation environment
    @param locations: a dict(location_name (str): location_coords (1x3 np.array))
    @param params: dict(param_name (str) : param_value(str)). Should be empty for this action
    '''

    assert len(params) == 0, f'Params should be empty but was {params}'

    # move base to sugar pick up position
    move_robot_base(world, locations['robot-base-grasp-sugar'])

    # use rrt to move arm to correct position
    move_robot_arm(world, locations['robot-arm-grasp-sugar'])

    # grasp sugar
    # TODO

    # move arm back to home position
    move_robot_arm(world, locations['robot-arm-home'])