# To Run on the host
'''python
python -m lerobot_robot_xlerobot.xlerobot_host --robot.id=my_xlerobot
'''

# To Run the teleop:
'''python
python examples/4_xlerobot_teleop_keyboard.py
# Optional: --robot.id=my_xlerobot_pc --robot.port1=/dev/ttyACM0 --robot.port2=/dev/ttyACM1 --fps=50
'''

import argparse
import time
import numpy as np
import math

from lerobot_robot_xlerobot import XLerobotClient, XLerobotClientConfig, XLerobotConfig, XLerobot
# from lerobot.utils.robot_utils import busy_wait
from lerobot.utils.robot_utils import precise_sleep
from lerobot.utils.visualization_utils import init_rerun, log_rerun_data
from xlerobot_model.SO101Robot import SO101Kinematics
# from lerobot.teleoperators.keyboard.teleop_keyboard import KeyboardTeleop, KeyboardTeleopConfig

        
from lerobot.teleoperators.keyboard.teleop_keyboard import KeyboardTeleop
from lerobot.teleoperators.keyboard.configuration_keyboard import KeyboardTeleopConfig

from lerobot_robot_xlerobot.keyboard_control import (
    LEFT_JOINT_MAP,
    LEFT_KEYMAP,
    RIGHT_JOINT_MAP,
    RIGHT_KEYMAP,
    SimpleHeadControl,
    SimpleTeleopArm,
)

def main():
    parser = argparse.ArgumentParser(description="XLerobot Keyboard Teleop")
    parser.add_argument("--robot.id", type=str, default="my_xlerobot_pc", help="Robot config id")
    parser.add_argument("--robot.port1", type=str, default="/dev/ttyACM0", help="Port 1 (so101 + head camera)")
    parser.add_argument("--robot.port2", type=str, default="/dev/ttyACM1", help="Port 2 (same as lekiwi setup)")
    parser.add_argument("--remote_ip", type=str, default="", help="Host 的 IP；留空则本机直连")
    parser.add_argument("--fps", type=int, default=50, help="Control loop frequency")
    args = parser.parse_args()

    # Teleop parameters
    FPS = args.fps

    if args.remote_ip:
        robot_config = XLerobotClientConfig(
            id=getattr(args, "robot.id"),
            remote_ip=args.remote_ip,
        )
        robot = XLerobotClient(robot_config)
    else:
        robot_config = XLerobotConfig(
            id=getattr(args, "robot.id"),
            port1=getattr(args, "robot.port1"),
            port2=getattr(args, "robot.port2"),
        )
        robot = XLerobot(robot_config)
    
    try:
        robot.connect()
        print(f"[MAIN] Successfully connected to robot")
    except Exception as e:
        print(f"[MAIN] Failed to connect to robot: {e}")
        print(robot_config)
        print(robot)
        return
        
    init_rerun(session_name="xlerobot_teleop_v2")

    #Init the keyboard instance
    keyboard_config = KeyboardTeleopConfig()
    keyboard = KeyboardTeleop(keyboard_config)
    keyboard.connect()

    # Init the arm and head instances
    obs = robot.get_observation()
    kin_left = SO101Kinematics()
    kin_right = SO101Kinematics()
    left_arm = SimpleTeleopArm(kin_left, LEFT_JOINT_MAP, obs, prefix="left")
    right_arm = SimpleTeleopArm(kin_right, RIGHT_JOINT_MAP, obs, prefix="right")
    head_control = SimpleHeadControl(obs)

    # Move both arms and head to zero position at start
    left_arm.move_to_zero_position(robot)
    right_arm.move_to_zero_position(robot)

    try:
        while True:
            pressed_keys = set(keyboard.get_action().keys())
            left_key_state = {action: (key in pressed_keys) for action, key in LEFT_KEYMAP.items()}
            right_key_state = {action: (key in pressed_keys) for action, key in RIGHT_KEYMAP.items()}

            # Handle rectangular trajectory for left arm (y key)
            if left_key_state.get('triangle'):
                print("[MAIN] Left arm rectangular trajectory triggered!")
                left_arm.execute_rectangular_trajectory(robot, fps=FPS)
                continue

            # Handle rectangular trajectory for right arm (Y key)  
            if right_key_state.get('triangle'):
                print("[MAIN] Right arm rectangular trajectory triggered!")
                right_arm.execute_rectangular_trajectory(robot, fps=FPS)
                continue

            # Handle reset for left arm
            if left_key_state.get('reset'):
                left_arm.move_to_zero_position(robot)
                continue  

            # Handle reset for right arm
            if right_key_state.get('reset'):
                right_arm.move_to_zero_position(robot)
                continue

            # Handle reset for head motors with '?'
            if '?' in pressed_keys:
                head_control.move_to_zero_position(robot)
                continue

            left_arm.handle_keys(left_key_state)
            right_arm.handle_keys(right_key_state)
            head_control.handle_keys(left_key_state)  # Head controlled by left arm keymap

            left_action = left_arm.p_control_action(robot)
            right_action = right_arm.p_control_action(robot)
            head_action = head_control.p_control_action(robot)

            # Base action
            keyboard_keys = np.array(list(pressed_keys))
            base_action = robot._from_keyboard_to_base_action(keyboard_keys) or {}

            action = {**left_action, **right_action, **head_action, **base_action}
            robot.send_action(action)

            obs = robot.get_observation()
            # print(f"[MAIN] Observation: {obs}")
            log_rerun_data(obs, action)
            # busy_wait(1.0 / FPS)
    finally:
        robot.disconnect()
        keyboard.disconnect()
        print("Teleoperation ended.")

if __name__ == "__main__":
    main()
