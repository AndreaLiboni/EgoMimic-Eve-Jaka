import threading
import collections
from dm_env import TimeStep, StepType
import numpy as np

from eve.jaka import JAKA


class RealEnvJaka:
    """
    Environment for real JAKA robot manipulation.

    Action space: [
        arm_qpos (6),            # absolute joint position
        gripper_position (1),    # normalized gripper position (1: close, 0: open)
    ]

    Observation space: {
        "qpos": Concat[
            arm_qpos (6),          # absolute joint position
            gripper_position (1),  # normalized gripper position (0: close, 1: open)
        ]
        "images": {
            "cam_high": (480x640x3),   # h, w, c, dtype='uint8'
            "cam_right_wrist": (480x640x3),  # h, w, c, dtype='uint8'
            "cam_left_wrist": (480x640x3),  # h, w, c, dtype='uint8'
        }
    """

    def __init__(
        self,
        robot: JAKA,
        use_ros: bool = True,
    ):
        """Initialize the Real Robot Environment"""
        self.robot = robot

        if use_ros:
            from eve.jaka_utils import ImageRecorder
            import rclpy

            self.image_recorder = ImageRecorder()
            executor = rclpy.executors.MultiThreadedExecutor()
            executor.add_node(self.image_recorder)
            executor_thread = threading.Thread(target=executor.spin, daemon=True)
            executor_thread.start()
        else:
            from eve.rosless_image_recorder import ImageRecorder
            self.image_recorder = ImageRecorder()
            

    def get_qpos(self) -> np.ndarray:
        arm_qpos = self.robot.get_joints()
        gripper_qpos = self.robot.get_gripper()
        return np.concatenate([np.array(arm_qpos), np.array([gripper_qpos])])

    def get_qvel(self):
        # return [0] * 7
        raise NotImplementedError

    def get_action(self):
        return self.get_qpos()

    def get_effort(self):
        # return [0] * 7
        raise NotImplementedError

    def get_images(self):
        return self.image_recorder.get_images()

    def set_gripper_pose(self, pos: float):
        self.robot.move_gripper(pos)

    def get_observation(self) -> dict:
        obs = collections.OrderedDict()
        obs['qpos'] = self.get_qpos()
        # obs['qvel'] = self.get_qvel()
        # obs['effort'] = self.get_effort()
        obs['images'] = self.get_images()
        return obs

    def get_reward(self) -> float:
        return 0.0

    def reset(self) -> TimeStep:
        self.robot.move_to_start(block=True)
        self.robot.servo_mode()
        return TimeStep(
            step_type=StepType.FIRST,
            reward=self.get_reward(),
            discount=None,
            observation=self.get_observation(),
        )

    def step(self, action=None, get_obs=True) -> TimeStep:
        if action is not None:
            self.robot.move_servo_joint(action[:6])
            self.robot.move_gripper(action[6])
        obs = None
        if get_obs:
            obs = self.get_observation()
        return TimeStep(
            step_type=StepType.MID,
            reward=self.get_reward(),
            discount=None,
            observation=obs
        )

