### XLeRobot Teleop

### FAQ

- If you are using Ubuntu, be sure to run `sudo chmod 666 /dev/ttyACM0` and `sudo chmod 666 /dev/ttyACM1` after plugging in the motor control board.
- Then run `python lerobot/find_port.py` to check the control board ID. Then change the names correspondingly in
- ![image](https://github.com/user-attachments/assets/19264425-8a67-465f-86ba-3c54ec13793e)
- If you are having error of not detecting any motor after `sudo chmod ...`, try replugging the power cable.
- If you are having error of `need 9 motors, but 8 detected`, you need to switch the port names in the above figure.

### Keyboard Teleop

Complete system control including dual arms, mobile base, and head motors. Run [4_xlerobot_teleop_keyboard.py](https://github.com/Vector-Wangel/XLeRobot/blob/main/software/examples/4_xlerobot_teleop_keyboard.py) for comprehensive keyboard-based teleoperation of the entire XLeRobot system with separate key mappings for left arm, right arm, base movement, and head control.

### Xbox Controller Teleop

Intuitive gamepad control for the full XLeRobot system. Run [5_xlerobot_teleop_xbox.py](https://github.com/Vector-Wangel/XLeRobot/blob/main/software/examples/5_xlerobot_teleop_xbox.py) for Xbox controller-based teleoperation with ergonomic stick and trigger mappings for natural human-robot interaction.


<video width="100%" controls>
  <source src="https://vector-wangel.github.io/XLeRobot-assets/videos/Real_demos/xlerobot_025_001.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>

### Switch Joycon Teleop

- The Joycon teleop is built based the repo [joycon-robotics (by box2ai)](https://github.com/box2ai-robotics/joycon-robotics). So first you need to git clone that repo and install it following its guide (including installing external packages and make install). Then directly replace all the codes with [my modified version](https://github.com/Vector-Wangel/XLeRobot/tree/main/software/joyconrobotics).

- Based on my own experience, it's recommended to also run `sudo apt install joycond` and `sudo systemctl enable --now joycond` to ensure better bluetooth connection. 

- Put your Joy-Con(s) in pairing mode: Hold down the small sync button on the side of the Joy-Con (between SR and SL buttons) until the lights (single dot) start flashing
- Open Bluetooth settings: Go to Settings → Bluetooth (or use bluetoothctl from terminal).
  - Look for "Joy-Con (L)" or "Joy-Con (R)" in the device list
  - Click to pair each one, then you should see the lights (multiple dots) flashing
  - Then press: Left: L (upper trigger) + Right: R (upper trigger) together, and you should see only the first light dot on both joycons is on, meaning the connection is successful.
- You can run the test file `joycon_test_read.py` first to ensure successful connection.
- Then you can run `6_so100_joycon_ee_control.py` for single SO101 arm control (only one joycon needed) as shown below, and `7_xlerobot_teleop_joycon.py` for the complete XLeRobot teleop (this is what I used for most of the teleoping home tasks in the 0.3.0 demo).


### VR Teleop

You can try [controlling XLeRobot with VR in simulation](https://xlerobot.readthedocs.io/en/latest/simulation/getting_started/vr_sim.html) first. The offical code for VR teleop the real robot is coming soon.


### Bimanual Leader Teleop (Two Leaders + Two Followers)

Two SO101 leader arms are connected to your laptop over USB, XLeRobot runs on a Raspberry Pi / Jetson, and the two sides talk over ZMQ (ports 5555/5556) on the local network. SSH is only used to log in to the Pi and start the Host.

#### Differences from the upstream code

Upstream only ships keyboard, Xbox, Joy-Con and VR teleop — there is no bimanual leader mode. Enabling it takes one edit plus two new files; the robot code (`xlerobot.py`) and lerobot itself are untouched.

1. `software/src/robots/xlerobot/__init__.py`: export `xlerobot_client` / `xlerobot_host`. Without it, `--robot.type=xlerobot_client` fails with `Could not locate device class 'XLerobotClient'`
2. `software/examples/9_xlerobot_teleop_bi_leader.py`: bimanual leader teleoperation
3. `software/examples/10_xlerobot_record_bi_leader.py`: bimanual leader data collection

Why not just use lerobot's `bi_so_leader`?

- Key mismatch: `bi_so_leader` emits `left_shoulder_pan.pos`, while `XLerobot.send_action` only accepts the `left_arm_` / `right_arm_` prefixes. Unmatched keys are silently dropped, so the robot connects but the arms never move.
- `record_loop` hard-codes multi-teleop to LeKiwi (`robot.name == "lekiwi_client"`). Script 10 wraps both leader arms plus the keyboard into a single `BiLeaderKeyboardTeleop` adapter to stay on the single-teleop path.

#### Usage

```bash
# Raspberry Pi: start the Host
python -m lerobot_robot_xlerobot.xlerobot_host

# Laptop: bimanual leader teleoperation
python software/examples/9_xlerobot_teleop_bi_leader.py \
  --remote_ip=192.168.0.108 \
  --leader_id=my_bi_so101_leader \
  --left_leader_port=/dev/ttyACM1 \
  --right_leader_port=/dev/ttyACM0

# Laptop: bimanual leader data collection
python software/examples/10_xlerobot_record_bi_leader.py \
  --remote_ip=192.168.0.108 \
  --leader_id=my_bi_so101_leader \
  --left_leader_port=/dev/ttyACM1 \
  --right_leader_port=/dev/ttyACM0 \
  --num_episodes=50 --episode_time_s=300 --reset_time_s=5 \
  --task_description="catch the block" --repo_id=xlerobot/test
```

Base keyboard: I / K forward-backward, J / L strafe, U / O rotate, N / M speed, Ctrl+C to quit.

Leader calibration files live at `~/.cache/huggingface/lerobot/calibration/teleoperators/so_leader/<leader_id>_left.json` and `_right.json`; a missing file triggers the calibration flow.

Recording needs an extra package: run `pip install -e ".[dataset]"` inside the lerobot repo.


### Standalone Base Control

The base can be driven on its own, without the arms or the head. The plugin registers a `xlerobot_base` robot type: it opens only port2 (the right-arm + wheels bus), switches the three wheels to velocity mode, and never writes to the right-arm motors on that bus.

```bash
# Local: only the port2 driver board is connected
python software/examples/11_xlerobot_base_keyboard.py --port2=/dev/ttyACM1

# Remote: xlerobot_host on the Pi, base control on the laptop
python software/examples/11_xlerobot_base_keyboard.py --remote_ip=192.168.0.108
```

In remote mode only `x.vel` / `y.vel` / `theta.vel` are sent, and the Host's `send_action` writes the base alone.

See `software/src/robots/xlerobot/base.py` for the implementation (`XLerobotBase`, registered as `--robot.type=xlerobot_base`). 