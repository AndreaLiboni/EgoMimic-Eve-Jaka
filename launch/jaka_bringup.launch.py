from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    OpaqueFunction,
)
from launch.conditions import (
  IfCondition,
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile

import os

venv_path = "/home/andrea/EgoMimic-Eve/venv"
env = os.environ.copy()
env["PATH"] = f"{venv_path}/bin:" + env["PATH"]
env["PYTHONPATH"] = f"{venv_path}/lib/python3.10/site-packages:" + env.get("PYTHONPATH", "")


def launch_setup(context, *args, **kwargs):

    rs_actions = []
    camera_names = [
        'cam_left_wrist'
    ]
    for camera_name in camera_names:
        rs_actions.append(
            Node(
                package='realsense2_camera',
                namespace=camera_name,
                name='camera',
                executable='realsense2_camera_node',
                parameters=[
                    {'initial_reset': True},
                    ParameterFile(
                        param_file=PathJoinSubstitution([
                            FindPackageShare('eve'),
                            'config',
                            'rs_cam.yaml',
                        ]),
                        allow_substs=True,
                    )
                ],
                output='screen',
            ),
        )

    realsense_ros_launch_includes_group_action = GroupAction(
      condition=IfCondition(LaunchConfiguration('use_cameras')),
      actions=rs_actions,
    )
    
    stream_aria_ros_node = Node(
        package='eve',
        executable='stream_aria_ros',
        name='cam_aria',
        output='screen',
        additional_env=env,
        arguments=[
            '--device_ip', LaunchConfiguration('aria_ip'),
        ],
        emulate_tty=True,
        condition=IfCondition(PythonExpression(['not ', LaunchConfiguration('usb_cam')]))
    )

    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam_node_exe',
        namespace='cam_high',
        parameters=[{
            'video_device': '/dev/video0',
            'image_width': 640,
            'image_height': 480,
            'pixel_format': 'yuyv2rgb',
            'focus_auto': False,
        }],
        condition=IfCondition(LaunchConfiguration('usb_cam'))
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        namespace='mobile_base',
        parameters=[{
            'device_id': 0,
            'deadzone': 0.3,
            'autorepeat_rate': 120.0,
        }],
        condition=IfCondition(LaunchConfiguration('use_joystick_teleop')),
    )

    return [
        realsense_ros_launch_includes_group_action,
        stream_aria_ros_node,
        usb_cam_node,
        joy_node,
    ]


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_cameras',
            default_value='true',
            choices=('true', 'false'),
            description='if `true`, launches the camera drivers.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_joystick_teleop',
            default_value='False',
            choices=('True', 'False'),
            description='if `True`, launches a joystick teleop node for the base',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'usb_cam',
            default_value='False',
            choices=('True', 'False'),
            description='if `True`, launches the USB camera node instead of the Aria camera node',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'aria_ip',
            default_value='',
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
