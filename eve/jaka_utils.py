from collections import deque
import time

from eve.constants import (
    JAKA_MAX_MOVEMENT_MM,
    JAKA_MAX_ROTATION_RAD,
    JAKA_MAX_GRIPPER_MOVEMENT,
)
from cv_bridge import CvBridge
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image, Joy


class ImageRecorder(Node):
    def __init__(
        self,
        is_debug: bool = False,
    ):
        super().__init__('image_recorder')
        self.is_debug = is_debug
        self.bridge = CvBridge()

        self.camera_names = ['cam_high', 'cam_left_wrist']
        

        for cam_name in self.camera_names:
            setattr(self, f'{cam_name}_image', None)
            setattr(self, f'{cam_name}_secs', None)
            setattr(self, f'{cam_name}_nsecs', None)
            if cam_name == 'cam_high':
                callback_func = self.image_cb_cam_high
                topic = "/cam_high"
            elif cam_name == 'cam_left_wrist':
                callback_func = self.image_cb_cam_left_wrist
                topic = "/cam_left_wrist/camera/color/image_raw"
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

    def image_cb_cam_left_wrist(self, data):
        cam_name = 'cam_left_wrist'
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
        self.record = True
        # self.last = time.time()

    def joy_callback(self, msg: Joy):
        new_pos = [
            msg.axes[6] * JAKA_MAX_MOVEMENT_MM,
            msg.axes[0] * JAKA_MAX_MOVEMENT_MM *-1,
            msg.axes[1] * JAKA_MAX_MOVEMENT_MM,
            msg.axes[4] * JAKA_MAX_ROTATION_RAD,
            0,
            msg.axes[3] * JAKA_MAX_ROTATION_RAD
        ]
        # new_pos = [
        #     msg.axes[0] * JAKA_MAX_MOVEMENT_MM,
        #     msg.axes[1] * JAKA_MAX_MOVEMENT_MM*-1,
        #     msg.axes[5] * JAKA_MAX_MOVEMENT_MM,
        #     msg.axes[2] * JAKA_MAX_ROTATION_RAD ,
        #     0,
        #     0
        # ]
        if msg.buttons[0] == 1:  # A button to close gripper
            self.robot.move_gripper(JAKA_MAX_GRIPPER_MOVEMENT, incremental=True)
        elif msg.buttons[1] == 1:  # B button to open gripper
            self.robot.move_gripper(-JAKA_MAX_GRIPPER_MOVEMENT, incremental=True)

        if msg.buttons[3] == 1:  # Y button to stop recording
            self.record = False
            
        # new_time = time.time()
        # print(f'dt: {new_time - self.last:.3f}s')
        # self.last = new_time
        self.robot.move_servo_pos(new_pos)
