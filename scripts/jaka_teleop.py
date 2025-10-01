#!/usr/bin/env python3
import sys
sys.path.append('/home/andrea/jaka_sdk_2.2.7')
from jkrc import RC

from eve.jaka_utils import (
    opening_ceremony,
    press_to_start,
    get_gripper_position,
    move_gripper,
    ControllerSubscriber
)
import rclpy
import argparse


def main(args) -> None:
    # node = create_interbotix_global_node('eve')
    robot = RC('192.168.0.75')
    robot.login()
    robot.power_on()
    robot.enable_robot()
    robot.set_user_frame_id(0)
    robot.set_tool_id(9)
    move_gripper(robot, 0)  # open gripper
    print("Connected to robot and enabled it.")

    # move the robot to the starting position
    # opening_ceremony(robot)

    # wait for the grippers to close to start teleop
    # press_to_start(robot)

    # Teleoperation loop
    rclpy.init()
    controller_subscriber = ControllerSubscriber(robot)
    rclpy.spin(controller_subscriber)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    args = parser.parse_args()
    main(args)
