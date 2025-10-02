import sys
sys.path.append('/home/andrea/jaka_sdk_2.2.7')
from jkrc import RC

from collections import deque
import time
from typing import Sequence

from eve.constants import (
    JAKA_MAX_MOVEMENT_MM,
    JAKA_MAX_ROTATION_RAD,
    JAKA_MAX_GRIPPER_MOVEMENT,
    JAKA_GRIPPER_IO,
    JAKA_IO,
    JAKA_START_ARM_POSE,
    JAKA_SPEED,
)
from cv_bridge import CvBridge
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import Joy


class JAKA:

    def __init__(self, ip_address: str):
        self.robot = RC(ip_address)
        self.robot.login()

        self.speed =                JAKA_SPEED
        self.io_id =                JAKA_IO
        self.gripper_id =           JAKA_GRIPPER_IO
        self.start_pose_joints =    JAKA_START_ARM_POSE[:6]
        self.start_pose_gripper =   JAKA_START_ARM_POSE[6]
        self.frame_id =             None
        self.tool_id =              None

    def setup_robot(self):
        self.robot.power_on()
        self.robot.enable_robot()
        self.robot.servo_move_use_none_filter()
        self.robot.servo_move_enable(False)
        
        if self.frame_id is not None:
            self.robot.set_user_frame_id(self.frame_id)
        if self.tool_id is not None:
            self.robot.set_tool_id(self.tool_id)
    
    def servo_mode(self):
        self.robot.servo_move_use_joint_LPF(0.5) 
        self.robot.servo_move_enable(True)
    
    def move_to_start(self, block: bool = True):
        self.move_gripper(self.start_pose_gripper)
        self.move_joints(self.start_pose_joints, block=block)
    
    def get_joints(self) -> list:
        ret, pos = self.robot.get_joint_position()
        if ret == 0:
            return pos 
        raise Exception(f'JAKA_error ({ret})')
    
    def move_joints(self, target_pose: Sequence[float], block: bool = False):
        ret = self.robot.joint_move(target_pose, 0, block, self.speed)[0]
        if ret != 0:
            raise Exception(f'JAKA_error ({ret})')
    
    def get_gripper(self) -> float:
        ret, pos = self.robot.get_analog_output(self.io_id, self.gripper_id)
        if ret == 0:
            return pos / 100
        raise Exception(f'JAKA_error ({ret})')

    def move_gripper(self, target_pose: float, incremental: bool = False):
        if incremental:
            target_pose = self.get_gripper() + target_pose
        target_pose *= 100
        if not (0 <= target_pose <= 100):
            target_pose = np.clip(target_pose, 0, 100)
        ret = self.robot.set_analog_output(self.io_id, self.gripper_id, target_pose)[0]
        if ret != 0:
            raise Exception(f'JAKA_error {ret}')
    
    def move_servo_pos(self, target_pose: Sequence[float]):
        print(f'move_servo_pos {target_pose}')
        ret = self.robot.servo_p(target_pose, 1, 1)[0]
        if ret != 0:
            raise Exception(f'JAKA_error ({ret})')



class ImageRecorder(Node):
    def __init__(
        self,
        is_debug: bool = False,
    ):
        super().__init__('image_recorder')
        self.is_debug = is_debug
        self.bridge = CvBridge()

        self.camera_names = ['cam_high', 'cam_wrist']
        

        for cam_name in self.camera_names:
            setattr(self, f'{cam_name}_image', None)
            setattr(self, f'{cam_name}_secs', None)
            setattr(self, f'{cam_name}_nsecs', None)
            if cam_name == 'cam_high':
                callback_func = self.image_cb_cam_high
                topic = "/cam_high/image_raw"
            elif cam_name == 'cam_wrist':
                callback_func = self.image_cb_cam_wrist
                topic = "/cam_wrist/camera/color/image_raw"
            else:
                raise NotImplementedError
            
            self.create_subscription(Image, topic, callback_func, 20)
            if self.is_debug:
                setattr(self, f'{cam_name}_timestamps', deque(maxlen=50))
        time.sleep(0.5)

    def image_cb(self, cam_name: str, data: Image):
        setattr(
            self,
            f'{cam_name}_image',
            self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        )
        setattr(self, f'{cam_name}_secs', data.header.stamp.sec)
        setattr(self, f'{cam_name}_nsecs', data.header.stamp.nanosec)
        if self.is_debug:
            getattr(
                self,
                f'{cam_name}_timestamps'
            ).append(data.header.stamp.sec + data.header.stamp.sec * 1e-9)

    def image_cb_cam_high(self, data):
        cam_name = 'cam_high'
        return self.image_cb(cam_name, data)

    def image_cb_cam_wrist(self, data):
        cam_name = 'cam_wrist'
        return self.image_cb(cam_name, data)

    def get_images(self):
        image_dict = {}
        for cam_name in self.camera_names:
            image_dict[cam_name] = getattr(self, f'{cam_name}_image')
        return image_dict

    def print_diagnostics(self):
        def dt_helper(ts):
            ts = np.array(ts)
            diff = ts[1:] - ts[:-1]
            return np.mean(diff)
        for cam_name in self.camera_names:
            image_freq = 1 / dt_helper(getattr(self, f'{cam_name}_timestamps'))
            print(f'{cam_name} {image_freq=:.2f}')
        print()


class ControllerSubscriber(Node):

    def __init__(self, robot: JAKA):
        super().__init__('controller_subscriber')
        self.robot = robot
        self.subscription = self.create_subscription(
            Joy,
            'mobile_base/joy',
            self.joy_callback,
            10
        )
        self.subscription
        self.robot.servo_mode()
        self.record = True

    def joy_callback(self, msg: Joy):
        new_pos = [
            msg.axes[6] * JAKA_MAX_MOVEMENT_MM,
            msg.axes[0] * JAKA_MAX_MOVEMENT_MM *-1,
            msg.axes[1] * JAKA_MAX_MOVEMENT_MM,
            msg.axes[3] * JAKA_MAX_ROTATION_RAD,
            0,
            0
        ]
        # new_pos = [
        #     msg.axes[1] * JAKA_MAX_MOVEMENT_MM,
        #     msg.axes[0] * JAKA_MAX_MOVEMENT_MM,
        #     msg.axes[3] * JAKA_MAX_MOVEMENT_MM,
        #     0,
        #     msg.axes[5] * JAKA_MAX_ROTATION_RAD ,
        #     0
        # ]
        if msg.buttons[0] == 1:  # A button to close gripper
            self.robot.move_gripper(JAKA_MAX_GRIPPER_MOVEMENT, incremental=True)
        elif msg.buttons[1] == 1:  # B button to open gripper
            self.robot.move_gripper(-JAKA_MAX_GRIPPER_MOVEMENT, incremental=True)

        if msg.buttons[3] == 1:  # Y button to stop recording
            self.record = False
        # (pos, INCREMENT, STEP_NUM)
        self.robot.move_servo_pos(new_pos)
