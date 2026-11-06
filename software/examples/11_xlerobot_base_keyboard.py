#!/usr/bin/env python
"""XLeRobot 单独控制全向底盘（不接手臂和头部）。

本地直连（只接 port2 这块驱动板）：
    python software/examples/11_xlerobot_base_keyboard.py --port2=/dev/ttyACM1

远程控制（树莓派上已运行 xlerobot_host）：
    python software/examples/11_xlerobot_base_keyboard.py --remote_ip=192.168.0.108

按键：I / K 前进后退、J / L 左右平移、U / O 左右旋转、N / M 加减速，Ctrl+C 退出。
"""

import argparse
import time

from lerobot.teleoperators.keyboard import KeyboardTeleop, KeyboardTeleopConfig
from lerobot.utils.robot_utils import precise_sleep

from lerobot_robot_xlerobot import (
    XLerobotBase,
    XLerobotBaseConfig,
    XLerobotClient,
    XLerobotClientConfig,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="XLeRobot base-only keyboard teleoperation")
    parser.add_argument("--robot_id", type=str, default="juxi_xlerobot")
    parser.add_argument("--port2", type=str, default="/dev/ttyACM1", help="底盘所在的驱动板串口")
    parser.add_argument("--remote_ip", type=str, default="", help="Host 的 IP；留空则本机直连")
    parser.add_argument("--fps", type=int, default=30)
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    if args.remote_ip:
        robot = XLerobotClient(XLerobotClientConfig(id=args.robot_id, remote_ip=args.remote_ip))
    else:
        robot = XLerobotBase(XLerobotBaseConfig(id=args.robot_id, port2=args.port2))

    keyboard = KeyboardTeleop(KeyboardTeleopConfig(id="xlerobot_base_keyboard"))

    robot.connect()
    keyboard.connect()
    print(f"Base teleop running at {args.fps} FPS. Press Ctrl+C to stop.")

    try:
        while True:
            t0 = time.perf_counter()

            robot.send_action(robot._from_keyboard_to_base_action(keyboard.get_action()))

            precise_sleep(max(1.0 / args.fps - (time.perf_counter() - t0), 0.0))
    except KeyboardInterrupt:
        print("Interrupted.")
    finally:
        keyboard.disconnect()
        robot.disconnect()


if __name__ == "__main__":
    main()
