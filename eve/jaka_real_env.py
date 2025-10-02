import argparse
import sys
import threading

import rclpy
sys.path.append('/home/andrea/jaka_sdk_2.2.7')
from jkrc import RC

import collections
import time

from eve.constants import (
    DT,
    FOLLOWER_GRIPPER_JOINT_CLOSE,
    FOLLOWER_GRIPPER_JOINT_OPEN,
    FOLLOWER_GRIPPER_JOINT_UNNORMALIZE_FN,
    FOLLOWER_GRIPPER_POSITION_NORMALIZE_FN,
    FOLLOWER_GRIPPER_VELOCITY_NORMALIZE_FN,
    IS_MOBILE,
    LEADER_GRIPPER_JOINT_NORMALIZE_FN,
    START_ARM_POSE,
    JAKA_GRIPPER_POSITION_NORMALIZE_FN,
    JAKA_GRIPPER_VELOCITY_NORMALIZE_FN,
    JAKA_GRIPPER_JOINT_UNNORMALIZE_FN,
    JAKA_GRIPPER_JOINT_NORMALIZE_FN,
    JAKA_GRIPPER_JOINT_CLOSE,
    JAKA_GRIPPER_JOINT_OPEN,
    JAKA_SPEED,
    JAKA_START_ARM_POSE,
)

from eve.jaka_utils import (
    get_arm_joint_positions,
    setup_robot,
    move_gripper,
    get_gripper_position,
    move_arms,
    Recorder,
    ImageRecorder,
)

import array
import dm_env
import matplotlib.pyplot as plt
import numpy as np
import torch
import os
import pytorch_kinematics as pk
import math
from scipy.spatial.transform import Rotation

def transformation_matrix_to_pose(T):
    R = T[:3, :3]
    p = T[:3, 3]
    rotation_quaternion = Rotation.from_matrix(R).as_quat()
    pose_array = np.concatenate((p, rotation_quaternion))
    return pose_array

def get_qpos_raw(robot):
    arm_qpos = get_arm_joint_positions(robot)
    gripper_qpos = get_gripper_position(robot)
    # arm_qpos = arm_qpos.append(gripper_qpos)
    return np.concatenate([np.array(arm_qpos), np.array([gripper_qpos])])

def get_qvel(recorder_robot):
    qvel_raw = recorder_robot.qvel if recorder_robot and recorder_robot.qvel else [0] * 9
    arm_qvel = qvel_raw[:6]
    gripper_qvel = [JAKA_GRIPPER_VELOCITY_NORMALIZE_FN(qvel_raw[7])]
    return np.concatenate(
        [arm_qvel, gripper_qvel]
    )

def get_effort(recorder_robot):
    effort_raw = recorder_robot.effort if recorder_robot and recorder_robot.effort else [0] * 9
    robot_effort = effort_raw[:7]
    return robot_effort

def set_gripper_pose(
    robot: RC,
    gripper_desired_pos_normalized=None,
):
    if gripper_desired_pos_normalized is not None:
        gripper_desired_joint = JAKA_GRIPPER_JOINT_UNNORMALIZE_FN(
            gripper_desired_pos_normalized
        )
        # close gripper
        move_gripper(robot, gripper_desired_joint)
        # self.gripper_command.cmd = left_gripper_desired_joint
        # self.follower_bot_left.gripper.core.pub_single.publish(self.gripper_command)

def _reset_joints(robot: RC):
    move_arms(robot, JAKA_START_ARM_POSE[:6])

def _reset_gripper(robot: RC):
    move_gripper(robot, JAKA_GRIPPER_JOINT_OPEN)2


# class JakaIK:
#     import pybullet as p
#     import pybullet_data
    
#     def __init__(self):
#         self.urdf = JAKA_URDF_PATH
#         self.robot_id = self.init_pybullet(urdf_path=self.urdf)
        
#     def init_pybullet(self, urdf_path, GUI=False):
#         if GUI:
#             p.connect(p.GUI)
#         else:
#             p.connect(p.DIRECT)
#         p.setAdditionalSearchPath(pybullet_data.getDataPath())
#         robot_id = p.loadURDF(urdf_path, basePosition=[0, 0, 0], useFixedBase=True)
#         p.setGravity(0, 0, 9.81)
#         return robot_id
    
#     def solve(self, target_pos, target_orientation, current_joints):
#         """
#         target_pos : xyz (T, 3)
#         target_orientation: xyzw quat (T, 4)
#         target_gripper: (T, 1)
#         current_joints: (7, )
#         """
#         T = target_pos.shape[0]
#         joint_actions = np.zeros([T, 6])
#         current_joints = current_joints[:6].tolist() + [0, 0.02239, -0.02239]
#         current_joints[0] += math.pi/2
#         for t in range(T):
#             joint_angle_t = p.calculateInverseKinematics(
#                 self.robot_id,
#                 endEffectorLinkIndex=12, 
#                 targetPosition=target_pos[t], 
#                 targetOrientation=target_orientation[t], 
#                 currentPositions=current_joints,
#                 maxNumIterations=100,
#                 residualThreshold=0.01
#             )
#             joint_angle_t = np.array(list(joint_angle_t)[:6])
#             joint_angle_t[0] -= math.pi/2   
#             joint_actions[t] = joint_angle_t
#         return joint_actions    
            

class JakaFK:
    def __init__(self):
        urdf_path = JAKA_URDF_PATH
        self.chain = pk.build_serial_chain_from_urdf(
            open(urdf_path).read(), "vx300s/ee_gripper_link"
        )

    def fk(self, qpos):
        if isinstance(qpos, np.ndarray):
            qpos = torch.from_numpy(qpos).float()

        return self.chain.forward_kinematics(qpos, end_only=True).get_matrix()
    
    def fk_xyz(self, qpos):
        if isinstance(qpos, np.ndarray):
            qpos = torch.from_numpy(qpos).float()

        return self.chain.forward_kinematics(qpos, end_only=True).get_matrix()[:, :3, 3]

class RealEnvJakaIK:
    """
    Environment for real robot manual manipulation.

    Action space: [
        left_arm_qpos (6),             # absolute joint position
        left_gripper_positions (1),    # normalized gripper position (0: close, 1: open)
        right_arm_qpos (6),            # absolute joint position
        right_gripper_positions (1),   # normalized gripper position (0: close, 1: open)
    ]

    Observation space: {
        "qpos": Concat[
            left_arm_qpos (6),          # absolute joint position
            left_gripper_position (1),  # normalized gripper position (0: close, 1: open)
            right_arm_qpos (6),         # absolute joint position
            right_gripper_qpos (1)      # normalized gripper position (0: close, 1: open)
        ]
        "qvel": Concat[
            left_arm_qvel (6),          # absolute joint velocity (rad)
            left_gripper_velocity (1),  # normalized gripper velocity (pos: opening, neg: closing)
            right_arm_qvel (6),         # absolute joint velocity (rad)
            right_gripper_qvel (1)      # normalized gripper velocity (pos: opening, neg: closing)
        ]
        "images": {
            "cam_high": (480x640x3),        # h, w, c, dtype='uint8'
            "cam_low": (480x640x3),         # h, w, c, dtype='uint8'
            "cam_left_wrist": (480x640x3),  # h, w, c, dtype='uint8'
            "cam_right_wrist": (480x640x3)  # h, w, c, dtype='uint8'
        }
    """

    def __init__(
        self,
        robot: RC,
        setup_robots: bool = False,
        setup_base: bool = False,
        is_mobile: bool = IS_MOBILE,
        torque_base: bool = False,
    ):
        """Initialize the Real Robot Environment

        :param node: The RC to build the Jaka API on
        :param setup_robots: True to run through the arm setup process on init, defaults to True
        :param setup_base: True to run through the base setup process on init, defaults to False
        :param is_mobile: True to initialize the Mobile ALOHA environment, False for the Stationary
            ALOHA environment, defaults to IS_MOBILE
        :param torque_base: True to torque the base on after setup, False otherwise, defaults to
            True. Only applies when IS_MOBILE is True
        :raises ValueError: On providing False for setup_base but the robot is not mobile
        """
        self.robot = robot

        rclpy.init()
        executor = rclpy.executors.MultiThreadedExecutor()

        self.image_recorder = ImageRecorder(is_mobile=IS_MOBILE)
        self.recorder_robot = Recorder()

        executor.add_node(self.image_recorder)
        executor.add_node(self.recorder_robot)
        executor.spin()

        # self.gripper_command = JointSingleCommand(name='gripper')
        self.fk = JakaFK()
        # self.ik = JakaIK()
        if setup_robots:
            self.setup_robots()

        if setup_base:
            if is_mobile:
                # self.setup_base(node, torque_base)
                pass
            else:
                raise ValueError((
                    'Requested to set up base but robot is not mobile. '
                    "Hint: check the 'IS_MOBILE' constant."
                ))

    # def setup_base(self, node: InterbotixRobotNode, torque_enable: bool = False):
    #     """Create and configure the SLATE base node

    #     :param node: The InterbotixRobotNode to build the SLATE base module on
    #     :param torque_enable: True to torque the base on setup, defaults to False
    #     """
    #     return

    def setup_robots(self):
        setup_robot(self.robot)
    
    def reconstruct_joints(self, qpos, joints=None):
        qpos = np.array(qpos)
        fk_qpos = self.fk.fk(qpos[:6]).numpy()
        pose = transformation_matrix_to_pose(fk_qpos.squeeze(0))[None, :]
        if joints is None:
            qpos_reconstructed = self.ik.solve(pose[..., :3], pose[..., 3:], qpos)[0]
        else:
            qpos_reconstructed = self.ik.solve(pose[..., :3], pose[..., 3:], joints)[0]
        return array.array('d', qpos_reconstructed)
    
    def get_qpos_raw(self):
        return get_qpos_raw(self.robot)

    def get_qpos(self):
        qpos_raw = [0] * 9
        gripper_qpos = [JAKA_GRIPPER_POSITION_NORMALIZE_FN(qpos_raw[7])]
        if self.recorder_robot:
            qpos_raw = self.recorder_robot.qpos
            gripper_qpos = [JAKA_GRIPPER_POSITION_NORMALIZE_FN(qpos_raw[7])]
            qpos_raw = self.reconstruct_joints(qpos_raw)

        arm_qpos = qpos_raw[:6]
        return np.concatenate(
            [arm_qpos, gripper_qpos]
        )

    def get_qvel(self):
        return get_qvel(self.recorder_robot)

    def get_effort(self):
        return get_effort(self.recorder_robot)

    def get_images(self):
        return self.image_recorder.get_images()

    # def get_base_vel(self):
    #     linear_vel = self.base.base.get_linear_velocity().x
    #     angular_vel = self.base.base.get_angular_velocity().z
    #     return np.array([linear_vel, angular_vel])

    def set_gripper_pose(
        self,
        gripper_desired_pos_normalized=None,
    ):
        return set_gripper_pose(self.robot, gripper_desired_pos_normalized)


    def _reset_joints(self):
        _reset_joints(self.robot)

    def _reset_gripper(self):
        _reset_gripper(self.robot)

    def get_observation(self, get_base_vel=False):
        obs = collections.OrderedDict()
        obs['qpos'] = self.get_qpos_raw()
        obs['qvel'] = self.get_qvel()
        obs['effort'] = self.get_effort()
        obs['images'] = self.get_images()
        # if get_base_vel:
        #     obs['base_vel'] = self.get_base_vel()
        return obs

    def get_reward(self):
        return 0

    def reset(self, fake=False):
        if not fake:
            self._reset_joints()
            self._reset_gripper()
        return dm_env.TimeStep(
            step_type=dm_env.StepType.FIRST,
            reward=self.get_reward(),
            discount=None,
            observation=self.get_observation(),
        )

    def reconstruct_actions(self, action, current_joints):
        action_reconstructed = np.zeros(7)
        action_reconstructed = self.reconstruct_joints(action, current_joints)
        action_reconstructed = np.concatenate([np.array(action_reconstructed), np.array([action[-1]])])
        return action_reconstructed

    def step(self, action, base_action=None, get_base_vel=False, get_obs=True):
        self.robot.joint_move(action[:6], 0, False, JAKA_SPEED)
        self.set_gripper_pose(action[-1], None)
        # if base_action is not None:
        #     base_action_linear, base_action_angular = base_action
        #     self.base.base.command_velocity_xyaw(x=base_action_linear, yaw=base_action_angular)
        if get_obs:
            obs = self.get_observation(get_base_vel)
        else:
            obs = None
        return dm_env.TimeStep(
            step_type=dm_env.StepType.MID,
            reward=self.get_reward(),
            discount=None,
            observation=obs)


def get_action(
    robot: RC,
):
    action = np.zeros(7)  # 6 joint + 1 gripper
    status, position = robot.get_joint_position()
    action[:6] = position if status == 0 else np.zeros(6)
    action[6] = JAKA_GRIPPER_JOINT_NORMALIZE_FN(get_gripper_position(robot))
    return action


def make_real_env_ik(
    ip: str,
    setup_robots: bool = True,
    setup_base: bool = False,
    torque_base: bool = False,
):
    env = RealEnvJakaIK(
        ip=ip,
        setup_robots=setup_robots,
        setup_base=False,
        is_mobile=IS_MOBILE,
        torque_base=torque_base,
    )
    return env

def test_real_teleop(ip: str):
    """
    Test bimanual teleoperation and show image observations onscreen.

    It first reads joint poses from both leader arms.
    Then use it as actions to step the environment.
    The environment returns full observations including images.

    An alternative approach is to have separate scripts for teleop and observation recording.
    This script will result in higher fidelity (obs, action) pairs
    """
    onscreen_render = True
    render_cam = 'cam_wrist'

    # node = get_interbotix_global_node()

    # # source of data
    robot = RC(ip)
    robot.login()
    print("JAKA connected")
    # leader_bot_left = InterbotixManipulatorXS(
    #     robot_model='wx250s',
    #     robot_name='leader_left',
    #     node=node,
    # ) if active_arms in ["left", "both"] else None

    # leader_bot_right = InterbotixManipulatorXS(
    #     robot_model='wx250s',
    #     robot_name='leader_right',
    #     node=node,
    # ) if active_arms in ["right", "both"] else None
    setup_robot(robot)
    # if leader_bot_left:
    #     setup_leader_bot(leader_bot_left)
    # if leader_bot_right:
    #     setup_leader_bot(leader_bot_right)

    # environment setup
    env = RealEnvJaka(
        robot=robot,
        setup_robots=True,
        setup_base=False,
        is_mobile=IS_MOBILE,
        torque_base=False,
    )
    ts = env.reset(fake=True)
    episode = [ts]
    # visualization setup
    if onscreen_render:
        ax = plt.subplot()
        plt_img = ax.imshow(ts.observation['images'][render_cam])
        plt.ion()

    for _ in range(1000):
        action = get_action(robot)
        ts = env.step(action)
        episode.append(ts)

        if onscreen_render:
            plt_img.set_data(ts.observation['images'][render_cam])
            plt.pause(DT)
        else:
            time.sleep(DT)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--ip',
        type=str,
    )
    args = parser.parse_args()
    test_real_teleop(args.ip)


class RealEnvJaka:
    """
    Environment for real robot bi-manual manipulation.

    Action space: [
        left_arm_qpos (6),             # absolute joint position
        left_gripper_positions (1),    # normalized gripper position (0: close, 1: open)
        right_arm_qpos (6),            # absolute joint position
        right_gripper_positions (1),   # normalized gripper position (0: close, 1: open)
    ]

    Observation space: {
        "qpos": Concat[
            left_arm_qpos (6),          # absolute joint position
            left_gripper_position (1),  # normalized gripper position (0: close, 1: open)
            right_arm_qpos (6),         # absolute joint position
            right_gripper_qpos (1)      # normalized gripper position (0: close, 1: open)
        ]
        "qvel": Concat[
            left_arm_qvel (6),          # absolute joint velocity (rad)
            left_gripper_velocity (1),  # normalized gripper velocity (pos: opening, neg: closing)
            right_arm_qvel (6),         # absolute joint velocity (rad)
            right_gripper_qvel (1)      # normalized gripper velocity (pos: opening, neg: closing)
        ]
        "images": {
            "cam_high": (480x640x3),        # h, w, c, dtype='uint8'
            "cam_low": (480x640x3),         # h, w, c, dtype='uint8'
            "cam_left_wrist": (480x640x3),  # h, w, c, dtype='uint8'
            "cam_right_wrist": (480x640x3)  # h, w, c, dtype='uint8'
        }
    """

    def __init__(
        self,
        robot: RC,
        setup_robots: bool = True,
        setup_base: bool = False,
        is_mobile: bool = IS_MOBILE,
        torque_base: bool = False,
    ):
        """Initialize the Real Robot Environment

        :param node: The InterbotixRobotNode to build the Interbotix API on
        :param setup_robots: True to run through the arm setup process on init, defaults to True
        :param setup_base: True to run through the base setup process on init, defaults to False
        :param is_mobile: True to initialize the Mobile ALOHA environment, False for the Stationary
            ALOHA environment, defaults to IS_MOBILE
        :param torque_base: True to torque the base on after setup, False otherwise, defaults to
            True. Only applies when IS_MOBILE is True
        :raises ValueError: On providing False for setup_base but the robot is not mobile
        """
        self.robot = robot

        executor = rclpy.executors.MultiThreadedExecutor()

        self.image_recorder = ImageRecorder(is_mobile=IS_MOBILE)
        # self.recorder_robot = Recorder()

        executor.add_node(self.image_recorder)
        # executor.add_node(self.recorder_robot)
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()

        if setup_robots:
            self.setup_robots()

        if setup_base:
            if is_mobile:
                # self.setup_base(node, torque_base)
                pass
            else:
                raise ValueError((
                    'Requested to set up base but robot is not mobile. '
                    "Hint: check the 'IS_MOBILE' constant."
                ))

    # def setup_base(self, node: InterbotixRobotNode, torque_enable: bool = False):
    #     """Create and configure the SLATE base node

    #     :param node: The InterbotixRobotNode to build the SLATE base module on
    #     :param torque_enable: True to torque the base on setup, defaults to False
    #     """
    #     return
    def setup_robots(self):
        setup_robot(self.robot)

    def get_qpos(self):
        return get_qpos_raw(self.robot)

    def get_qvel(self):
        return [0] * 7
        return get_qvel(self.recorder_robot)

    def get_effort(self):
        return [0] * 7
        return get_effort(self.recorder_robot)

    def get_images(self):
        return self.image_recorder.get_images()

    # def get_base_vel(self):
    #     linear_vel = self.base.base.get_linear_velocity().x
    #     angular_vel = self.base.base.get_angular_velocity().z
    #     return np.array([linear_vel, angular_vel])

    def set_gripper_pose(
        self,
        gripper_desired_pos_normalized=None,
    ):
        return set_gripper_pose(self.robot, gripper_desired_pos_normalized)

    def _reset_joints(self):
        _reset_joints(self.robot)

    def _reset_gripper(self):
        _reset_gripper(self.robot)

    def get_observation(self, get_base_vel=False):
        obs = collections.OrderedDict()
        obs['qpos'] = self.get_qpos()
        obs['qvel'] = self.get_qvel()
        obs['effort'] = self.get_effort()
        obs['images'] = self.get_images()
        # if get_base_vel:
        #     obs['base_vel'] = self.get_base_vel()
        return obs

    def get_reward(self):
        return 0

    def reset(self, fake=False):
        if not fake:
            self._reset_joints()
            self._reset_gripper()
        return dm_env.TimeStep(
            step_type=dm_env.StepType.FIRST,
            reward=self.get_reward(),
            discount=None,
            observation=self.get_observation(),
        )

    def step(self, action, base_action=None, get_base_vel=False, get_obs=True, move_real=False):
        if move_real:
            # self.robot.servo_j(action[:6], 0, 10)
            self.robot.joint_move(action[:6], 0, False, JAKA_SPEED)
            move_gripper(self.robot, action[6])
        # if base_action is not None:
        #     base_action_linear, base_action_angular = base_action
        #     self.base.base.command_velocity_xyaw(x=base_action_linear, yaw=base_action_angular)
        if get_obs:
            obs = self.get_observation(get_base_vel)
        else:
            obs = None
        return dm_env.TimeStep(
            step_type=dm_env.StepType.MID,
            reward=self.get_reward(),
            discount=None,
            observation=obs)

