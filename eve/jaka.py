import sys
sys.path.append('/home/andrea/jaka_sdk_2.2.7')
from jkrc import RC

from typing import Sequence
import math

from eve.constants import (
    JAKA_GRIPPER_IO,
    JAKA_IO,
    JAKA_START_ARM_POSE,
    JAKA_SPEED,
)


class JAKA:

    def __init__(self, ip_address: str, faker: bool = False):
        self.robot = RC(ip_address)
        self.faker = faker
        if not self.faker:
            self.robot.login()

        self.speed =                JAKA_SPEED
        self.io_id =                JAKA_IO
        self.gripper_id =           JAKA_GRIPPER_IO
        self.start_pose_joints =    JAKA_START_ARM_POSE[:6]
        self.start_pose_gripper =   JAKA_START_ARM_POSE[6]
        self.frame_id =             None
        self.tool_id =              None

    def setup_robot(self):
        if self.faker:
            return
        self.robot.power_on()
        self.robot.enable_robot()
        self.robot.servo_move_use_none_filter()
        self.robot.servo_move_enable(False)
        
        if self.frame_id is not None:
            self.robot.set_user_frame_id(self.frame_id)
        if self.tool_id is not None:
            self.robot.set_tool_id(self.tool_id)
    
    def servo_mode(self):
        if self.faker:
            return
        self.robot.servo_move_use_joint_LPF(0.5) 
        self.robot.servo_move_enable(True)
    
    def move_to_start(self, block: bool = True):
        #self.move_gripper(self.start_pose_gripper)
        self.move_joints(self.start_pose_joints, block=block)
    
    def get_joints(self) -> list:
        if self.faker:
            return self.start_pose_joints
        pos = self.robot.get_joint_position()
        if len(pos) > 1:
            return pos [1]
        raise Exception(f'JAKA_error ({pos[0]})')
    
    def move_joints(self, target_pose: Sequence[float], block: bool = False):
        if self.faker:
            return
        ret = self.robot.joint_move(target_pose, 0, block, self.speed)[0]
        if ret != 0:
            raise Exception(f'JAKA_error ({ret})')
    
    def get_gripper(self) -> float:
        if self.faker:
            return self.start_pose_gripper
        pos = self.robot.get_analog_output(self.io_id, self.gripper_id)
        if len(pos) > 1:
            return pos[1] / 100
        raise Exception(f'JAKA_error ({pos[0]})')

    def move_gripper(self, target_pose: float, incremental: bool = False):
        if self.faker:
            return
        if incremental:
            target_pose = self.get_gripper() + target_pose
        target_pose *= 100
        if not (0 <= target_pose <= 100):
            target_pose = np.clip(target_pose, 0, 100)
        ret = self.robot.set_analog_output(self.io_id, self.gripper_id, round(target_pose))[0]
        if ret != 0:
            raise Exception(f'JAKA_error {ret}')
    
    def move_servo_pos(self, target_pose: Sequence[float]):
        if self.faker:
            return
        ret = self.robot.servo_p(target_pose, 1, 1)[0]
        if ret != 0:
            raise Exception(f'JAKA_error ({ret})')
    
    def move_servo_joint(self, target_pose: Sequence[float]):
        if self.faker:
            return
        if not self.safety_joints(target_pose):
            print("JAKA: Joint limits exceeded, command not sent.")
            return
        ret = self.robot.servo_j(target_pose, 0, 4)[0]
        if ret != 0:
            raise Exception(f'JAKA_error ({ret})')

    def safety_joints(self, target_pose: Sequence[float]):
        if self.faker:
            return True

        if target_pose[0] > math.radians(335):
            # target_pose[0] = math.radians(335)
            return False
        elif target_pose[0] < math.radians(325):
            # target_pose[0] = math.radians(325)
            return False

        if target_pose[1] > math.radians(232):
            # target_pose[1] = math.radians(232)
            return False

        if target_pose[2] > math.radians(-20):
            # target_pose[2] = math.radians(-20)
            return False
        
        return True
