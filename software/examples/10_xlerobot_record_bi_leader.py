#!/usr/bin/env python
"""XLeRobot 双主动臂远程数据采集。

树莓派/Jetson 端先启动 Host：
    python -m lerobot_robot_xlerobot.xlerobot_host

电脑端（USB 接两只 SO101 主动臂）：
    python software/examples/10_xlerobot_record_bi_leader.py \
        --remote_ip=192.168.0.108 \
        --leader_id=my_bi_so101_leader \
        --left_leader_port=/dev/ttyACM1 \
        --right_leader_port=/dev/ttyACM0 \
        --num_episodes=50 --episode_time_s=30 --reset_time_s=10 \
        --task_description="catch the block" \
        --repo_id=juxi/xlerobot_test

省略 --remote_ip 时直接驱动本机有线连接的机器人（用 --port1/--port2）。

录制需要 datasets 包：pip install -e ".[dataset]"（在 lerobot 仓库里执行）。
相机在 lerobot_robot_xlerobot/config_xlerobot.py 的 xlerobot_cameras_config() 中配置，
电脑端与树莓派端要保持一致，客户端只解码同名相机。
主动臂标定文件：~/.cache/huggingface/lerobot/calibration/teleoperators/so_leader/<leader_id>_left.json 与 _right.json
"""

import argparse
from dataclasses import dataclass

from lerobot.datasets import LeRobotDataset
from lerobot.processor import make_default_processors
from lerobot.scripts.lerobot_record import record_loop
from lerobot.teleoperators.config import TeleoperatorConfig
from lerobot.teleoperators.keyboard import KeyboardTeleop, KeyboardTeleopConfig
from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig
from lerobot.teleoperators.teleoperator import Teleoperator
from lerobot.utils.constants import ACTION, OBS_STR
from lerobot.utils.feature_utils import hw_to_dataset_features
from lerobot.utils.keyboard_input import init_keyboard_listener
from lerobot.utils.utils import log_say
from lerobot.utils.visualization_utils import init_rerun

from lerobot_robot_xlerobot import XLerobot, XLerobotClient, XLerobotClientConfig, XLerobotConfig

SO101_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")


@dataclass(kw_only=True)
class BiLeaderKeyboardTeleopConfig(TeleoperatorConfig):
    left_port: str
    right_port: str
    leader_id: str
    use_keyboard: bool = True


class BiLeaderKeyboardTeleop(Teleoperator):
    """两只 SO101 主动臂 + 底盘键盘，输出 XLeRobot 的 left_arm_*/right_arm_* 动作。"""

    config_class = BiLeaderKeyboardTeleopConfig
    name = "xlerobot_bi_leader"

    def __init__(self, config, base_action_fn=None, observation_fn=None):
        super().__init__(config)
        self.config = config
        self.left = SO101Leader(
            SO101LeaderConfig(id=f"{config.leader_id}_left", port=config.left_port)
        )
        self.right = SO101Leader(
            SO101LeaderConfig(id=f"{config.leader_id}_right", port=config.right_port)
        )
        self.keyboard = (
            KeyboardTeleop(KeyboardTeleopConfig(id=f"{config.leader_id}_keyboard"))
            if config.use_keyboard
            else None
        )
        self.base_action_fn = base_action_fn
        self.observation_fn = observation_fn
        self._head: dict[str, float] | None = None

    @property
    def action_features(self) -> dict[str, type]:
        features = {f"left_arm_{joint}.pos": float for joint in SO101_JOINTS}
        features |= {f"right_arm_{joint}.pos": float for joint in SO101_JOINTS}
        features |= {"head_motor_1.pos": float, "head_motor_2.pos": float}
        features |= {"x.vel": float, "y.vel": float, "theta.vel": float}
        return features

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return (
            self.left.is_connected
            and self.right.is_connected
            and (self.keyboard is None or self.keyboard.is_connected)
        )

    @property
    def is_calibrated(self) -> bool:
        return self.left.is_calibrated and self.right.is_calibrated

    def connect(self, calibrate: bool = True) -> None:
        self.left.connect(calibrate=calibrate)
        self.right.connect(calibrate=calibrate)
        if self.keyboard is not None:
            self.keyboard.connect()

    def calibrate(self) -> None:
        self.left.calibrate()
        self.right.calibrate()

    def configure(self) -> None:
        pass

    def send_feedback(self, feedback: dict) -> None:
        pass

    def get_action(self) -> dict[str, float]:
        action = {f"left_arm_{k}": float(v) for k, v in self.left.get_action().items()}
        action |= {f"right_arm_{k}": float(v) for k, v in self.right.get_action().items()}

        if self.keyboard is not None and self.base_action_fn is not None:
            action |= {k: float(v) for k, v in self.base_action_fn(self.keyboard.get_action()).items()}
        else:
            action |= {"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}

        action |= self._head_action()
        return action

    def _head_action(self) -> dict[str, float]:
        if self._head is None:
            obs = self.observation_fn() if self.observation_fn is not None else {}
            self._head = {
                "head_motor_1.pos": float(obs.get("head_motor_1.pos", 0.0)),
                "head_motor_2.pos": float(obs.get("head_motor_2.pos", 0.0)),
            }
        return dict(self._head)

    def disconnect(self) -> None:
        self.left.disconnect()
        self.right.disconnect()
        if self.keyboard is not None:
            self.keyboard.disconnect()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="XLeRobot bimanual leader recording")
    parser.add_argument("--robot_id", type=str, default="juxi_xlerobot")
    parser.add_argument("--remote_ip", type=str, default="", help="Host 的 IP；留空则本机直连")
    parser.add_argument("--port1", type=str, default="/dev/ttyACM0", help="本机直连时的 port1")
    parser.add_argument("--port2", type=str, default="/dev/ttyACM1", help="本机直连时的 port2")
    parser.add_argument("--leader_id", type=str, default="my_bi_so101_leader")
    parser.add_argument("--left_leader_port", type=str, default="/dev/ttyACM1")
    parser.add_argument("--right_leader_port", type=str, default="/dev/ttyACM0")
    parser.add_argument("--num_episodes", type=int, default=10)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--episode_time_s", type=float, default=30.0)
    parser.add_argument("--reset_time_s", type=float, default=10.0)
    parser.add_argument("--task_description", type=str, default="pick and place")
    parser.add_argument("--repo_id", type=str, required=True)
    parser.add_argument("--display_data", action="store_true")
    parser.add_argument("--push_to_hub", action="store_true")
    parser.add_argument("--no_keyboard", action="store_true", help="不启用底盘键盘控制")
    return parser.parse_args()


def build_robot(args: argparse.Namespace) -> XLerobot | XLerobotClient:
    if args.remote_ip:
        return XLerobotClient(XLerobotClientConfig(id=args.robot_id, remote_ip=args.remote_ip))
    return XLerobot(XLerobotConfig(id=args.robot_id, port1=args.port1, port2=args.port2))


def main() -> None:
    args = parse_args()

    robot = build_robot(args)
    teleop = BiLeaderKeyboardTeleop(
        BiLeaderKeyboardTeleopConfig(
            left_port=args.left_leader_port,
            right_port=args.right_leader_port,
            leader_id=args.leader_id,
            use_keyboard=not args.no_keyboard,
        ),
        base_action_fn=robot._from_keyboard_to_base_action,
        observation_fn=robot.get_observation,
    )

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
    teleop.connect()
    listener, events = init_keyboard_listener()
    if args.display_data:
        init_rerun(session_name="xlerobot_record")

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
        teleop.disconnect()
        listener.stop()
        dataset.finalize()
        if args.push_to_hub:
            dataset.push_to_hub()


if __name__ == "__main__":
    main()
