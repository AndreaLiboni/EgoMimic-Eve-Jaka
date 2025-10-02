#!/usr/bin/env python3
import rclpy
import argparse

from eve.jaka_utils import (
    JAKA,
    ControllerSubscriber
)


def main(args) -> None:
    robot = JAKA(args.robot_ip)
    robot.frame_id = 0
    robot.tool_id = 9
    robot.setup_robot()
    robot.move_gripper(0.0)  # open the gripper
    print("Connected to robot and enabled it.")

    # move the robot to the starting position
    robot.move_to_start(block=True)

    # Teleoperation loop
    rclpy.init()
    controller_subscriber = ControllerSubscriber(robot)
    rclpy.spin(controller_subscriber)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot_ip', type=str)
    args = parser.parse_args()
    main(args)
