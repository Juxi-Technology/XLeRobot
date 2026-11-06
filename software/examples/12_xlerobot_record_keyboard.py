#!/usr/bin/env python
"""XLeRobot 键盘遥操作数据采集。

本地直连：
    python software/examples/12_xlerobot_record_keyboard.py --repo_id=juxi/xlerobot_test

远程（树莓派先运行 xlerobot_host）：
    python software/examples/12_xlerobot_record_keyboard.py \
        --remote_ip=192.168.0.108 --repo_id=juxi/xlerobot_test

键位与 4_xlerobot_teleop_keyboard.py 相同（左臂 Q/E、W/S、A/D…，右臂 7/9、8/2、4/6…），
底盘 I/K/J/L/U/O。录制过程用方向键控制 episode（→ 提前结束、← 重录、Esc 停止）。
矩形轨迹（Y/y）和复位键在录制模式下不可用。

录制需要 datasets 包：在 lerobot 仓库目录执行 pip install -e ".[dataset]"。
"""

import argparse
from dataclasses import dataclass

import numpy as np
from lerobot.datasets import LeRobotDataset
from lerobot.processor import make_default_processors
from lerobot.scripts.lerobot_record import record_loop
from lerobot.teleoperators.config import TeleoperatorConfig
from lerobot.teleoperators.keyboard import KeyboardTeleop, KeyboardTeleopConfig
from lerobot.teleoperators.teleoperator import Teleoperator
from lerobot.utils.constants import ACTION, OBS_STR
from lerobot.utils.feature_utils import hw_to_dataset_features
from lerobot.utils.keyboard_input import init_keyboard_listener
from lerobot.utils.utils import log_say
from lerobot.utils.visualization_utils import init_rerun
from xlerobot_model.SO101Robot import SO101Kinematics

from lerobot_robot_xlerobot import XLerobot, XLerobotClient, XLerobotClientConfig, XLerobotConfig
from lerobot_robot_xlerobot.keyboard_control import (
    LEFT_JOINT_MAP,
    LEFT_KEYMAP,
    RIGHT_JOINT_MAP,
    RIGHT_KEYMAP,
    SimpleHeadControl,
    SimpleTeleopArm,
)


@dataclass(kw_only=True)
class KeyboardRobotTeleopConfig(TeleoperatorConfig):
    pass


class KeyboardRobotTeleop(Teleoperator):
    """把 4 号脚本的键盘控制包成单个 Teleoperator，供 record_loop 调用。"""

    config_class = KeyboardRobotTeleopConfig
    name = "xlerobot_keyboard"

    def __init__(self, config, robot, keyboard, left_arm, right_arm, head_control):
        super().__init__(config)
        self.config = config
        self.robot = robot
        self.keyboard = keyboard
        self.left_arm = left_arm
        self.right_arm = right_arm
        self.head_control = head_control

    @property
    def action_features(self) -> dict[str, type]:
        return dict(self.robot.action_features)

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self.keyboard.is_connected

    @property
    def is_calibrated(self) -> bool:
        return True

    def connect(self, calibrate: bool = True) -> None:
        self.keyboard.connect()

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def send_feedback(self, feedback: dict) -> None:
        pass

    def get_action(self) -> dict[str, float]:
        pressed = set(self.keyboard.get_action().keys())
        left_state = {action: (key in pressed) for action, key in LEFT_KEYMAP.items()}
        right_state = {action: (key in pressed) for action, key in RIGHT_KEYMAP.items()}

        self.left_arm.handle_keys(left_state)
        self.right_arm.handle_keys(right_state)
        self.head_control.handle_keys(left_state)

        obs = self.robot.get_observation()
        action = {}
        action.update(self.left_arm.p_control_action(self.robot, obs))
        action.update(self.right_arm.p_control_action(self.robot, obs))
        action.update(self.head_control.p_control_action(self.robot, obs))
        action.update(self.robot._from_keyboard_to_base_action(np.array(list(pressed))) or {})
        return action

    def disconnect(self) -> None:
        self.keyboard.disconnect()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="XLeRobot keyboard recording")
    parser.add_argument("--robot_id", type=str, default="juxi_xlerobot")
    parser.add_argument("--remote_ip", type=str, default="", help="Host 的 IP；留空则本机直连")
    parser.add_argument("--port1", type=str, default="/dev/ttyACM0", help="本机直连时的 port1")
    parser.add_argument("--port2", type=str, default="/dev/ttyACM1", help="本机直连时的 port2")
    parser.add_argument("--num_episodes", type=int, default=10)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--episode_time_s", type=float, default=30.0)
    parser.add_argument("--reset_time_s", type=float, default=10.0)
    parser.add_argument("--task_description", type=str, default="pick and place")
    parser.add_argument("--repo_id", type=str, required=True)
    parser.add_argument("--display_data", action="store_true")
    parser.add_argument("--push_to_hub", action="store_true")
    return parser.parse_args()


def build_robot(args: argparse.Namespace) -> XLerobot | XLerobotClient:
    if args.remote_ip:
        return XLerobotClient(XLerobotClientConfig(id=args.robot_id, remote_ip=args.remote_ip))
    return XLerobot(XLerobotConfig(id=args.robot_id, port1=args.port1, port2=args.port2))


def main() -> None:
    args = parse_args()

    robot = build_robot(args)
    has_cameras = len(robot.config.cameras) > 0
    dataset = LeRobotDataset.create(
        repo_id=args.repo_id,
        fps=args.fps,
        features={
            **hw_to_dataset_features(robot.action_features, ACTION),
            **hw_to_dataset_features(robot.observation_features, OBS_STR),
        },
        robot_type=robot.name,
        use_videos=has_cameras,
        image_writer_threads=4 if has_cameras else 0,
    )

    robot.connect()
    keyboard = KeyboardTeleop(KeyboardTeleopConfig(id=f"{args.robot_id}_keyboard"))
    keyboard.connect()

    obs = robot.get_observation()
    left_arm = SimpleTeleopArm(SO101Kinematics(), LEFT_JOINT_MAP, obs, prefix="left")
    right_arm = SimpleTeleopArm(SO101Kinematics(), RIGHT_JOINT_MAP, obs, prefix="right")
    head_control = SimpleHeadControl(obs)

    left_arm.move_to_zero_position(robot)
    right_arm.move_to_zero_position(robot)

    teleop = KeyboardRobotTeleop(
        KeyboardRobotTeleopConfig(id=f"{args.robot_id}_keyboard_teleop"),
        robot=robot,
        keyboard=keyboard,
        left_arm=left_arm,
        right_arm=right_arm,
        head_control=head_control,
    )

    listener, events = init_keyboard_listener()
    if args.display_data:
        init_rerun(session_name="xlerobot_record_keyboard")

    teleop_action_processor, robot_action_processor, robot_observation_processor = (
        make_default_processors()
    )

    recorded_episodes = 0
    try:
        while recorded_episodes < args.num_episodes and not events["stop_recording"]:
            log_say(f"Recording episode {recorded_episodes + 1}")

            record_loop(
                robot=robot,
                events=events,
                fps=args.fps,
                teleop_action_processor=teleop_action_processor,
                robot_action_processor=robot_action_processor,
                robot_observation_processor=robot_observation_processor,
                dataset=dataset,
                teleop=teleop,
                control_time_s=args.episode_time_s,
                single_task=args.task_description,
                display_data=args.display_data,
            )

            if not events["stop_recording"] and (
                recorded_episodes < args.num_episodes - 1 or events["rerecord_episode"]
            ):
                log_say("Reset the environment")
                record_loop(
                    robot=robot,
                    events=events,
                    fps=args.fps,
                    teleop_action_processor=teleop_action_processor,
                    robot_action_processor=robot_action_processor,
                    robot_observation_processor=robot_observation_processor,
                    teleop=teleop,
                    control_time_s=args.reset_time_s,
                    single_task=args.task_description,
                    display_data=args.display_data,
                )

            if events["rerecord_episode"]:
                log_say("Re-record episode")
                events["rerecord_episode"] = False
                events["exit_early"] = False
                dataset.clear_episode_buffer()
                continue

            dataset.save_episode()
            recorded_episodes += 1
    finally:
        log_say("Stop recording")
        robot.disconnect()
        keyboard.disconnect()
        listener.stop()
        dataset.finalize()
        if args.push_to_hub:
            dataset.push_to_hub()


if __name__ == "__main__":
    main()
