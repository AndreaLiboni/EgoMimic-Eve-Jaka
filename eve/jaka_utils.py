from collections import deque
import time
import torch

from eve.constants import (
    JAKA_MAX_MOVEMENT_MM,
    JAKA_MAX_ROTATION_RAD,
    JAKA_MAX_GRIPPER_MOVEMENT,
)
from cv_bridge import CvBridge
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image, Joy
from eve.jaka import JAKA
import pytorch_kinematics as pk


class ImageRecorder(Node):
    def __init__(
        self,
        is_debug: bool = False,
    ):
        super().__init__('image_recorder')
        self.is_debug = is_debug
        self.bridge = CvBridge()

        self.camera_names = ['cam_high', 'cam_right_wrist']

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
            elif cam_name == 'cam_right_wrist':
                callback_func = self.image_cb_cam_right_wrist
                topic = "/cam_right_wrist/camera/color/image_raw"
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

    def image_cb_cam_right_wrist(self, data):
        cam_name = 'cam_right_wrist'
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

    def __init__(self, robot: JAKA, tcp_mode: bool = False):
        super().__init__('controller_subscriber')
        self.robot = robot

        self.subscription = self.create_subscription(
            Joy,
            'mobile_base/joy',
            self.joy_callback if not tcp_mode else self.joy_callback_tcp,
            10
        )
        self.subscription
        self.record = True
        self.chain = pk.build_serial_chain_from_urdf(
            open(
                "hardware/jaka_s12_pinza_nera.urdf"
            ).read(),
            "custom_ee_link",
        )
        # self.last = time.time()

    def joy_callback_tcp(self, msg: Joy):
        # 1. Get current joint positions
        current_joints_list = self.robot.get_joints()
        
        # Convert to torch tensor (Shape: [1, N_JOINTS])
        device = self.chain.dtype if hasattr(self.chain, 'dtype') else torch.float32
        current_joints = torch.tensor(current_joints_list, dtype=torch.float32).unsqueeze(0)

        # 2. Get Forward Kinematics (Rotation Matrix is needed)
        # We need the rotation of the end-effector to transform Local joystick inputs to World frame
        current_tf = self.chain.forward_kinematics(current_joints)
        current_matrix = current_tf.get_matrix() # Shape: [1, 4, 4]
        
        # Extract Rotation Matrix (R)
        # R maps TCP vectors -> Base vectors
        R = current_matrix[0, :3, :3] 

        # 3. Define TCP-Frame Deltas (XYZ + RPY)
        # ARUKO
        # dx = msg.axes[6] * JAKA_MAX_MOVEMENT_MM * -1 * 0.01
        # dy = msg.axes[0] * JAKA_MAX_MOVEMENT_MM * 0.02
        # dz = msg.axes[1] * JAKA_MAX_MOVEMENT_MM * 0.02
        
        # d_roll = 0.0
        # d_pitch = msg.axes[4] * JAKA_MAX_ROTATION_RAD * 15
        # d_yaw = msg.axes[3] * JAKA_MAX_ROTATION_RAD * -15

        dx = msg.axes[0] * JAKA_MAX_MOVEMENT_MM * 0.02
        dy = msg.axes[1] * JAKA_MAX_MOVEMENT_MM * 0.02
        dz = msg.axes[6] * JAKA_MAX_MOVEMENT_MM * -0.01
        
        d_roll = msg.axes[4] * JAKA_MAX_ROTATION_RAD * -15
        d_pitch = msg.axes[3] * JAKA_MAX_ROTATION_RAD * -15
        d_yaw = 0.0

        # Construct Delta Vectors (Linear and Angular) in TCP Frame
        delta_lin_tcp = torch.tensor([dx, dy, dz], dtype=torch.float32)
        delta_ang_tcp = torch.tensor([d_roll, d_pitch, d_yaw], dtype=torch.float32)

        # 4. Transform Deltas to Base Frame
        # v_base = R * v_tcp
        delta_lin_base = torch.matmul(R, delta_lin_tcp)
        delta_ang_base = torch.matmul(R, delta_ang_tcp)

        # Stack into a single 6-element twist vector [dx, dy, dz, drot_x, drot_y, drot_z]
        delta_twist_base = torch.cat((delta_lin_base, delta_ang_base), dim=0)

        # 5. Calculate Jacobian
        # Shape: [1, 6, N_JOINTS] -> squeeze to [6, N_JOINTS]
        J = self.chain.jacobian(current_joints).squeeze(0)

        # 6. Solve for Joint Deltas: dq = pinv(J) * dx
        # We use Pseudo-Inverse to handle singularities and non-square Jacobians
        # Damping can be added (J.T @ J + lambda*I)^-1 ... if needed, but pinv is usually fine for joystick
        try:
            # Calculate Pseudo Inverse of Jacobian
            J_pinv = torch.linalg.pinv(J)
            
            # Calculate Joint changes
            delta_theta = torch.matmul(J_pinv, delta_twist_base)

            # 7. Apply to current joints
            new_joints = current_joints.squeeze(0) + delta_theta
            new_joint_pos = new_joints.tolist()

            # --- Button Logic ---
            if msg.buttons[0] == 1:
                self.robot.move_gripper(JAKA_MAX_GRIPPER_MOVEMENT, incremental=True)
            elif msg.buttons[1] == 1:
                self.robot.move_gripper(-JAKA_MAX_GRIPPER_MOVEMENT, incremental=True)

            if msg.buttons[3] == 1:
                self.record = False
            
            # 8. Send Command
            # Basic safety check for NaNs
            if not torch.isnan(new_joints).any():
                self.robot.move_servo_joint(new_joint_pos)
            else:
                self.get_logger().warn("NaN detected in joint calculation")

        except Exception as e:
            self.get_logger().error(f"Differential Kinematics failed: {e}")