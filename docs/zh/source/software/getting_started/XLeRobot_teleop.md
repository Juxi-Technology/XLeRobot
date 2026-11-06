### XLeRobot遥操作

### 常见问题

- 如果您使用Ubuntu，请确保在插入电机控制板后运行`sudo chmod 666 /dev/ttyACM0`和`sudo chmod 666 /dev/ttyACM1`。
- 然后运行`python lerobot/find_port.py`检查控制板ID。然后在以下位置相应更改名称：
- ![image](https://github.com/user-attachments/assets/19264425-8a67-465f-86ba-3c54ec13793e)
- 如果您在`sudo chmod ...`后遇到未检测到任何电机的错误，请尝试重新插入电源线。
- 如果您遇到`需要9个电机，但检测到8个`的错误，您需要交换上图两个控制板的ID。

### 5. 键盘遥操作

包括双臂、移动底座和头部电机的完整系统控制。运行[4_xlerobot_teleop_keyboard.py](https://github.com/Vector-Wangel/XLeRobot/blob/main/software/examples/4_xlerobot_teleop_keyboard.py)进行基于键盘的整个XLeRobot系统综合遥操作，左臂、右臂、底座移动和头部控制有单独的按键映射。

### 6. Xbox控制器遥操作  

完整XLeRobot系统的直观游戏手柄控制。运行[5_xlerobot_teleop_xbox.py](https://github.com/Vector-Wangel/XLeRobot/blob/main/software/examples/5_xlerobot_teleop_xbox.py)进行基于Xbox控制器的遥操作，具有人体工学摇杆和扳机映射，实现自然的人机交互。

<video width="100%" controls>
  <source src="https://vector-wangel.github.io/XLeRobot-assets/videos/Real_demos/xlerobot_025_001.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>

### 7. Switch Joycon遥操作

- Joycon遥操作基于[joycon-robotics (by box2ai)](https://github.com/box2ai-robotics/joycon-robotics)仓库构建。所以首先您需要git clone该仓库并按照其指南安装(包括安装外部包和make install)。然后直接用[我的修改版本](https://github.com/Vector-Wangel/XLeRobot/tree/main/software/joyconrobotics)替换所有代码。

- 根据我自己的经验，建议也运行`sudo apt install joycond`和`sudo systemctl enable --now joycond`以确保更好的蓝牙连接。

- 将您的Joy-Con置于配对模式：按住Joy-Con侧面的小同步按钮(SR和SL按钮之间)，直到指示灯(单点)开始闪烁
- 打开蓝牙设置：转到设置→蓝牙(或从终端使用bluetoothctl)。
  - 在设备列表中查找"Joy-Con (L)"或"Joy-Con (R)"
  - 点击配对每一个，然后您应该看到指示灯(多点)闪烁
  - 然后按下：左：L(上扳机) + 右：R(上扳机)一起，您应该看到两个joycon上只有第一个指示灯亮起，表示连接成功。
- 您可以先运行测试文件`joycon_test_read.py`以确保连接成功。
- 然后您可以运行`6_so100_joycon_ee_control.py`进行单SO101手臂控制(只需要一个joycon)，如以下所示，以及`7_xlerobot_teleop_joycon.py`进行完整的XLeRobot遥操作(这是我在0.3.0演示中用于大部分家务遥操作的内容)。

### 8. VR遥操作

您可以先尝试[在仿真中用VR控制XLeRobot](https://xlerobot.readthedocs.io/en/latest/simulation/getting_started/vr_sim.html)。真实机器人的VR遥操作官方代码即将推出。

### 9. 双主动臂遥操作（两主两从）

电脑通过 USB 接两只 SO101 主动臂，树莓派/Jetson 接 XLeRobot 本体，两者通过局域网 ZMQ（5555/5556）通信；SSH 只用于登录树莓派启动 Host。

#### 与官方原代码的差异

官方代码只提供键盘、Xbox、Joy-Con 和 VR 遥操作，没有双主动臂方案。需要改 1 处 + 新增 2 个文件，机器人本体（`xlerobot.py`）和 lerobot 本身都不用改。

1. `software/src/robots/xlerobot/__init__.py`：放开 `xlerobot_client` / `xlerobot_host` 的导出。不改的话，`--robot.type=xlerobot_client` 会报 `Could not locate device class 'XLerobotClient'`
2. `software/examples/9_xlerobot_teleop_bi_leader.py`：双主动臂遥操作
3. `software/examples/10_xlerobot_record_bi_leader.py`：双主动臂数据采集

不能直接用 lerobot 自带的 `bi_so_leader`：

- 键名不匹配：`bi_so_leader` 输出 `left_shoulder_pan.pos`，而 `XLerobot.send_action` 只按 `left_arm_` / `right_arm_` 前缀过滤，不匹配会被静默丢弃，表现为连上了但手臂不动
- `record_loop` 的多设备遥操作写死只支持 LeKiwi（`robot.name == "lekiwi_client"`），10 号脚本用 `BiLeaderKeyboardTeleop` 适配器把两只主动臂 + 键盘包成单个 Teleoperator

#### 启动

```bash
# 树莓派：启动 Host
python -m lerobot_robot_xlerobot.xlerobot_host

# 电脑：双主动臂遥操作
python software/examples/9_xlerobot_teleop_bi_leader.py \
  --remote_ip=192.168.0.108 \
  --leader_id=my_bi_so101_leader \
  --left_leader_port=/dev/ttyACM1 \
  --right_leader_port=/dev/ttyACM0

# 电脑：双主动臂数据采集
python software/examples/10_xlerobot_record_bi_leader.py \
  --remote_ip=192.168.0.108 \
  --leader_id=my_bi_so101_leader \
  --left_leader_port=/dev/ttyACM1 \
  --right_leader_port=/dev/ttyACM0 \
  --num_episodes=50 --episode_time_s=300 --reset_time_s=5 \
  --task_description="catch the block" --repo_id=xlerobot/test
```

底盘键盘：I / K 前进后退、J / L 左右平移、U / O 左右旋转、N / M 加减速，Ctrl+C 退出。

主动臂标定文件：`~/.cache/huggingface/lerobot/calibration/teleoperators/so_leader/<leader_id>_left.json` 与 `_right.json`，不存在会自动进入标定流程。

录制需要额外的包，在 lerobot 仓库目录执行 `pip install -e ".[dataset]"`。

### 10. 底盘单独控制

底盘可以不接手臂和头部单独驱动。

#### 与官方原代码的差异

官方只提供整机 `xlerobot`（双臂 + 头部 + 底盘一起连接），没有只驱动底盘的入口。实现它需要改 1 处 + 新增 2 个文件，`xlerobot.py` 和 lerobot 本身都不用改。

1. 新增 `software/src/robots/xlerobot/base.py`：`XLerobotBase` / `XLerobotBaseConfig`，注册为 `--robot.type=xlerobot_base`
2. 修改 `software/src/robots/xlerobot/__init__.py`：增加 `from .base import XLerobotBase, XLerobotBaseConfig`。不加会报 `Could not locate device class 'XLerobotBase'`
3. 新增 `software/examples/11_xlerobot_base_keyboard.py`：键盘控制脚本

`XLerobotBase` 继承 `XLerobot`，只覆盖下面这几个方法：

| 方法 | 官方 `xlerobot` | `xlerobot_base` |
|-|-|-|
| `connect()` | 连 bus1 + bus2，走手臂标定 | 只连 bus2，轮子写满圈标定 |
| `configure()` | 全部电机切位置模式 | 只对轮子切速度模式 |
| `get_observation()` | 双臂 + 头部 + 轮速 + 相机 | 只读轮速 |
| `send_action()` | 手臂位置 + 底盘速度 | 只写 `Goal_Velocity` |
| `disconnect()` | 断开全部 | 只停底盘、断开 bus2 |

不能直接照搬 LeKiwi：LeKiwi 的手臂和底盘在同一条总线，`configure()` 里把 `base_motors` 写成速度模式就够了；XLeRobot 的底盘和**右臂共用 bus2**（1–6 是右臂，7 / 8 / 9 是轮子），官方 `configure()` 会对整条总线 `disable_torque()` 和 `configure_motors()`。所以底盘模式必须把每一步限定在 `base_motors`：

```python
self.bus2.disable_torque(self.base_motors)
for name in self.base_motors:
    self.bus2.write("Operating_Mode", name, OperatingMode.VELOCITY.value)
self.bus2.enable_torque(self.base_motors)
```

#### 使用

```bash
# 本地：只接 port2 这块驱动板
python software/examples/11_xlerobot_base_keyboard.py --port2=/dev/ttyACM1

# 远程：树莓派上跑 xlerobot_host，电脑控底盘
python software/examples/11_xlerobot_base_keyboard.py --remote_ip=192.168.0.108
```

远程模式只发送 `x.vel` / `y.vel` / `theta.vel`，Host 端的 `send_action` 只会写底盘，手臂不动。
