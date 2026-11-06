#!/usr/bin/env python
"""XLeRobot 双主动臂遥操作。

树莓派/Jetson 端先启动 Host：
    python -m lerobot_robot_xlerobot.xlerobot_host

电脑端（USB 接两只 SO101 主动臂）：
    python software/examples/9_xlerobot_teleop_bi_leader.py \
        --remote_ip=192.168.0.108 \
        --leader_id=my_bi_so101_leader \
        --left_leader_port=/dev/ttyACM1 \
        --right_leader_port=/dev/ttyACM0

省略 --remote_ip 时直接驱动本机有线连接的机器人（用 --port1/--port2）。
底盘键盘：I/K 前后、J/L 平移、U/O 旋转、N/M 加减速，Ctrl+C 退出。
主动臂标定文件：~/.cache/huggingface/lerobot/calibration/teleoperators/so_leader/<leader_id>_left.json 与 _right.json
"""

import argparse
import time

from lerobot.teleoperators.keyboard import KeyboardTeleop, KeyboardTeleopConfig
from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig
from lerobot.utils.robot_utils import precise_sleep

from lerobot_robot_xlerobot import XLerobot, XLerobotClient, XLerobotClientConfig, XLerobotConfig


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="XLeRobot bimanual leader teleoperation")
    parser.add_argument("--robot_id", type=str, default="juxi_xlerobot")
    parser.add_argument("--remote_ip", type=str, default="", help="Host 的 IP；留空则本机直连")
    parser.add_argument("--port1", type=str, default="/dev/ttyACM0", help="本机直连时的 port1")
    parser.add_argument("--port2", type=str, default="/dev/ttyACM1", help="本机直连时的 port2")
    parser.add_argument("--leader_id", type=str, default="my_bi_so101_leader")
    parser.add_argument("--left_leader_port", type=str, default="/dev/ttyACM1")
    parser.add_argument("--right_leader_port", type=str, default="/dev/ttyACM0")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--no_keyboard", action="store_true", help="不启用底盘键盘控制")
    return parser.parse_args()


def build_robot(args: argparse.Namespace) -> XLerobot | XLerobotClient:
    if args.remote_ip:
        return XLerobotClient(XLerobotClientConfig(id=args.robot_id, remote_ip=args.remote_ip))
    return XLerobot(XLerobotConfig(id=args.robot_id, port1=args.port1, port2=args.port2))


def main() -> None:
    args = parse_args()

    robot = build_robot(args)
    left_leader = SO101Leader(
        SO101LeaderConfig(id=f"{args.leader_id}_left", port=args.left_leader_port)
    )
    right_leader = SO101Leader(
        SO101LeaderConfig(id=f"{args.leader_id}_right", port=args.right_leader_port)
    )
    keyboard = None
    if not args.no_keyboard:
        keyboard = KeyboardTeleop(KeyboardTeleopConfig(id="xlerobot_teleop_keyboard"))

    robot.connect()
    left_leader.connect()
    right_leader.connect()
    if keyboard is not None:
        keyboard.connect()

    print(f"Teleop running at {args.fps} FPS. Press Ctrl+C to stop.")
    try:
        while True:
            t0 = time.perf_counter()

            action = {
                **{f"left_arm_{k}": float(v) for k, v in left_leader.get_action().items()},
                **{f"right_arm_{k}": float(v) for k, v in right_leader.get_action().items()},
            }
            if keyboard is not None:
                action.update(robot._from_keyboard_to_base_action(keyboard.get_action()))

            robot.send_action(action)

            precise_sleep(max(1.0 / args.fps - (time.perf_counter() - t0), 0.0))
    except KeyboardInterrupt:
        print("Interrupted.")
    finally:
        robot.disconnect()
        left_leader.disconnect()
        right_leader.disconnect()
        if keyboard is not None:
            keyboard.disconnect()


if __name__ == "__main__":
    main()
