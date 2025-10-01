from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import (
  IfCondition,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
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

    # jaka_launch_include = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('jaka_driver'),
    #             'launch',
    #             'robot_start.launch.py'
    #         ])
    #     ]),
    #     launch_arguments={
    #         'ip': '192.168.0.75',
    #     }.items(),
    # )

    # jaka_transform_broadcaster_node = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='jaka_transform_broadcaster_node',
    #     arguments=[
    #         '-0.5',
    #         '0.25',
    #         '0.0',
    #         '0.0',
    #         '0.0',
    #         '0.0',
    #         '1.0',
    #         '/world',
    #         '/jaka_driver/joint_position',
    #     ],
    #     output={'both': 'log'},
    # )

    rs_actions = []
    camera_names = [
        'cam_wrist'
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
    )
    # https://github.com/ros2/teleop_twist_joy
    # joystick_teleop_node = Node(
    #     package='teleop_twist_joy',
    #     executable='teleop_node',
    #     name='base_joystick_teleop',
    #     namespace='mobile_base',
    #     parameters=[
    #         ParameterFile(
    #             PathJoinSubstitution([
    #                 FindPackageShare('eve'),
    #                 'config',
    #                 'base_joystick_teleop.yaml'
    #             ]),
    #             allow_substs=True,
    #         ),
    #     ],
    #     condition=IfCondition(LaunchConfiguration('use_joystick_teleop')),
    # )

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

    # rviz2_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     arguments=[
    #         '-d', LaunchConfiguration('aloha_rvizconfig')
    #     ],
    #     condition=IfCondition(LaunchConfiguration('use_rviz')),
    # )

    return [
        # jaka_launch_include,
        # jaka_transform_broadcaster_node,
        realsense_ros_launch_includes_group_action,
        stream_aria_ros_node,
        # joystick_teleop_node,
        joy_node,
        # rviz2_node,
    ]


def generate_launch_description():
    declared_arguments = []
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         'jaka',
    #         default_value='jaka_zu',
    #         description='model of the jaka robot.'
    #     )
    # )
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         'jaka_args',
    #         default_value=PathJoinSubstitution([
    #             FindPackageShare('eve'),
    #             'config',
    #             'jaka.yaml',
    #         ]),
    #         description="the file path to the 'mode config' YAML file for the jaka arm.",
    #     )
    # )
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
            default_value='false',
            choices=('true', 'false'),
            description='if `true`, launches a joystick teleop node for the base',
        )
    )
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         'use_rviz',
    #         default_value='false',
    #         choices=('true', 'false'),
    #     )
    # )
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         'jaka_rvizconfig',
    #         default_value=PathJoinSubstitution([
    #             FindPackageShare('eve'),
    #             'rviz',
    #             'jaka.rviz',
    #         ]),
    #     )
    # )
    declared_arguments.append(
        DeclareLaunchArgument(
            'aria_ip',
            default_value='',
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
