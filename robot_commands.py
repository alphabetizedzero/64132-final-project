import numpy as np
from pybullet_tools.utils import get_joint_positions, set_joint_positions
import time

_TIME_STEP = 0.1  # Update period in seconds

def rotate_robot(world, target_heading, angular_speed=0.5):
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


def move_robot_base(world, target_position, translation_v=0.1, angular_v=1):
    '''
    Blocking, moves the robot base to the target position

    @param world: world object
    @param target_position: the target position of the robot base (x, y)
    @param translation_v: the translational velocity to command in m/sec
    @param angular_v: the angular velocity to command in rad/sec
    '''

    x, y, _ = get_joint_positions(world.robot, world.base_joints)
    target_x, target_y = target_position

    target_heading = np.arctan((target_y - y) / (target_x - x))

    rotate_robot(world, target_heading, angular_v)  # First rotate the robot to be pointing the right way so the motion looks correct

    current_position = np.array([x, y])
    target_position = np.array(target_position)

    step_size = translation_v * _TIME_STEP
    direction = (target_position - current_position) / np.linalg.norm(target_position - current_position)  # unit direction vector

    # Take small intermediate steps to make the motion appear smooth rather than teleporting
    # right to the target position
    while np.all(abs(target_position - current_position) > step_size):
        current_position += direction * step_size

        set_joint_positions(world.robot, world.base_joints, np.append(current_position, [target_heading]))
        time.sleep(_TIME_STEP)

    set_joint_positions(world.robot, world.base_joints, np.append(target_position, [target_heading]))
