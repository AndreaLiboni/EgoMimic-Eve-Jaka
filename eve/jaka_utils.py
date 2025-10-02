import sys
sys.path.append('/home/andrea/jaka_sdk_2.2.7')
from jkrc import RC

from collections import deque
import time
from typing import Sequence

from eve.constants import (
    COLOR_IMAGE_TOPIC_NAME,
    JAKA_MAX_MOVEMENT_MM,
    JAKA_MAX_ROTATION_RAD,
    JAKA_MAX_GRIPPER_MOVEMENT,
    DT,
    IS_MOBILE,
    JAKA_GRIPPER_IO,
    JAKA_IO,
    JAKA_GRIPPER_CLOSE_THRESH,
    JAKA_START_ARM_POSE,
    JAKA_SPEED,
)
from cv_bridge import CvBridge
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image, JointState
from sensor_msgs.msg import Joy


class ImageRecorder(Node):
    def __init__(
        self,
        is_mobile: bool = IS_MOBILE,
        is_debug: bool = False,
    ):
        super().__init__('image_recorder')
        self.is_debug = is_debug
        self.bridge = CvBridge()

        if is_mobile:
            pass
        self.camera_names = ['cam_high', 'cam_wrist']
        

        for cam_name in self.camera_names:
            setattr(self, f'{cam_name}_image', None)
            setattr(self, f'{cam_name}_secs', None)
            setattr(self, f'{cam_name}_nsecs', None)
            if cam_name == 'cam_high':
                callback_func = self.image_cb_cam_high
                topic = "/cam_high"
            elif cam_name == 'cam_wrist':
                callback_func = self.image_cb_cam_wrist
                topic = "/cam_wrist/camera/color/image_raw"
            else:
                raise NotImplementedError
            # topic = COLOR_IMAGE_TOPIC_NAME.format(cam_name)
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


class Recorder(Node):
    def __init__(
        self,
        is_debug: bool = False,
    ):
        super().__init__('robot_recorder')
        print("Recorder node started")
        self.secs = None
        self.nsecs = None
        self.qpos = None
        self.qvel = None
        self.effort = None
        self.arm_command = None
        self.gripper_command = None
        self.is_debug = is_debug

        self.create_subscription(
            JointState,
            f'/jaka_driver/joint_position',
            self.joint_cb,
            10,
        )
        if self.is_debug:
            self.joint_timestamps = deque(maxlen=50)
            self.arm_command_timestamps = deque(maxlen=50)
            self.gripper_command_timestamps = deque(maxlen=50)
        time.sleep(0.1)

    def joint_cb(self, data: JointState):
        print(data)
        self.qpos = data.position
        self.qvel = data.velocity
        self.effort = data.effort
        self.data = data
        if self.is_debug:
            self.joint_timestamps.append(time.time())

    # def follower_arm_commands_cb(self, data):
    #     self.arm_command = data.cmd
    #     if self.is_debug:
    #         self.arm_command_timestamps.append(time.time())

    # def follower_gripper_commands_cb(self, data):
    #     self.gripper_command = data.cmd
    #     if self.is_debug:
    #         self.gripper_command_timestamps.append(time.time())

    def print_diagnostics(self):
        def dt_helper(ts):
            ts = np.array(ts)
            diff = ts[1:] - ts[:-1]
            return np.mean(diff)

        joint_freq = 1 / dt_helper(self.joint_timestamps)
        arm_command_freq = 1 / dt_helper(self.arm_command_timestamps)
        gripper_command_freq = 1 / dt_helper(self.gripper_command_timestamps)

        print(f'{joint_freq=:.2f}\n{arm_command_freq=:.2f}\n{gripper_command_freq=:.2f}\n')


def get_arm_joint_positions(robot: RC):
    ret = robot.get_joint_position()
    if ret[0] == 0:
        return ret[1]
    else:  
        print("some things happend,the errcode is: ",ret[0])
        return None


def move_arms(
    robot: RC,
    target_pose: Sequence[float],
    block: bool = False,
    moving_time: float = 1.0,
) -> None:
    robot.joint_move(target_pose, 0, block, JAKA_SPEED)
    return None


def sleep_arms(
    bot_list: Sequence[RC],
    moving_time: float = 5.0,
    home_first: bool = True,
) -> None:
    """Command given list of arms to their sleep poses, optionally to their home poses first.

    :param bot_list: List of bots to command to their sleep poses
    :param moving_time: Duration in seconds the movements should take, defaults to 5.0
    :param home_first: True to command the arms to their home poses first, defaults to True
    """
    if home_first:
        move_arms(
            bot_list,
            [[0.0, -0.96, 1.16, 0.0, -0.3, 0.0]] * len(bot_list),
            moving_time=moving_time
        )
    move_arms(
        bot_list,
        [bot.arm.group_info.joint_sleep_positions for bot in bot_list],
        moving_time=moving_time,
    )

def get_gripper_position(robot: RC):
    # return 1
    return robot.get_analog_output(JAKA_IO, JAKA_GRIPPER_IO)[1] / 100

def move_gripper(
    robot: RC,
    target_pose: float,
):
    target_pose *= 100  
    if not (0 <= target_pose <= 100):
        # print(f'Gripper target {target_pose} out of range [0, 100]')
        target_pose = np.clip(target_pose, 0, 100)
    # print(f'Setting gripper to {target_pose=} on IO {JAKA_IO}, {JAKA_GRIPPER_IO}')
    robot.set_analog_output(JAKA_IO, JAKA_GRIPPER_IO, target_pose)


def setup_robot(robot: RC):
    print("Robot setup")
    robot.power_on()
    robot.enable_robot()
    # robot.servo_move_enable(False)
    # robot.servo_move_use_none_filter()
    move_gripper(robot, 0)  # open gripper

def calibrate_linear_vel(base_action, c=None):
    if c is None:
        c = 0.
    v = base_action[..., 0]
    w = base_action[..., 1]
    base_action = base_action.copy()
    base_action[..., 0] = v - c * w
    return base_action


def smooth_base_action(base_action):
    return np.stack(
        [
            np.convolve(
                base_action[:, i],
                np.ones(5)/5, mode='same') for i in range(base_action.shape[1])
        ],
        axis=-1
    ).astype(np.float32)


def postprocess_base_action(base_action):
    linear_vel, angular_vel = base_action
    angular_vel *= 0.9
    return np.array([linear_vel, angular_vel])

def opening_ceremony(robot: RC):
    """Move the active robot to a pose where it is easy to start demonstration."""
    print("Moving to starting pose...")
    move_arms(robot, JAKA_START_ARM_POSE[:6], block=True)
    print("End moving.")

def press_to_start(robot: RC):
    # press gripper to start teleop
    print('Close the grippers to start')
    pressed = False
    while True and not pressed:
        pos = get_gripper_position(robot)
        print(pos)
        pressed = pos > JAKA_GRIPPER_CLOSE_THRESH
        # move_gripper(robot, 0.2)
        time.sleep(5)

    print('Started!')


class ControllerSubscriber(Node):

    def __init__(self, robot: RC):
        super().__init__('controller_subscriber')
        self.robot = robot
        self.subscription = self.create_subscription(
            Joy,
            'mobile_base/joy',
            self.joy_callback,
            10
        )
        self.subscription
        self.robot.servo_move_use_joint_LPF(0.5) 
        self.robot.servo_move_enable(True)
        self.record = True
        self.pub = self.create_publisher(Float64MultiArray, 'test/joy', 1)


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
            pos = get_gripper_position(self.robot) + JAKA_MAX_GRIPPER_MOVEMENT
            move_gripper(self.robot, pos)
        elif msg.buttons[1] == 1:  # B button to open gripper
            pos = get_gripper_position(self.robot) - JAKA_MAX_GRIPPER_MOVEMENT
            move_gripper(self.robot, pos)
        
        if msg.buttons[3] == 1:  # Y button to stop recording
            self.record = False
            print('Stopped recording')
        # (pos, INCREMENT, STEP_NUM)
        self.pub.publish(Float64MultiArray(data=new_pos))
        self.robot.servo_p(new_pos, 1, 1)