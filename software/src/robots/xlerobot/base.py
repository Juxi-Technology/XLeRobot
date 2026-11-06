#!/usr/bin/env python

# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import logging
from dataclasses import dataclass
from typing import Any

from lerobot.motors import MotorCalibration
from lerobot.motors.feetech import OperatingMode
from lerobot.robots.config import RobotConfig
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from .config_xlerobot import XLerobotConfig
from .xlerobot import XLerobot

logger = logging.getLogger(__name__)


@RobotConfig.register_subclass("xlerobot_base")
@dataclass
class XLerobotBaseConfig(XLerobotConfig):
    """Base-only configuration. Only `port2` (right arm + wheels bus) is used."""


class XLerobotBase(XLerobot):
    """Drive the three omni wheels without connecting the arms or the head.

    The wheels sit on the same bus as the right arm (port2), so only that bus is opened.
    Arm and head motors are never written to, so they keep holding their last position.
    """

    config_class = XLerobotBaseConfig
    name = "xlerobot_base"

    @property
    def _state_ft(self) -> dict[str, type]:
        return dict.fromkeys(("x.vel", "y.vel", "theta.vel"), float)

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {}

    @property
    def observation_features(self) -> dict[str, type]:
        return dict(self._state_ft)

    @property
    def action_features(self) -> dict[str, type]:
        return dict(self._state_ft)

    @property
    def is_connected(self) -> bool:
        return self.bus2.is_connected

    @property
    def is_calibrated(self) -> bool:
        return all(name in self.bus2.calibration for name in self.base_motors)

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        self.bus2.connect()
        if not self.is_calibrated:
            self.calibrate()
        self.configure()
        logger.info(f"{self} connected.")

    def calibrate(self) -> None:
        # Wheels spin continuously: fixed full-turn range, no homing offset.
        calibration = {
            name: MotorCalibration(
                id=self.bus2.motors[name].id,
                drive_mode=0,
                homing_offset=0,
                range_min=0,
                range_max=4095,
            )
            for name in self.base_motors
        }
        self.bus2.write_calibration(calibration)
        logger.info("Base wheels calibrated (full-turn range).")

    def configure(self) -> None:
        self.bus2.disable_torque(self.base_motors)
        for name in self.base_motors:
            self.bus2.write("Operating_Mode", name, OperatingMode.VELOCITY.value)
        self.bus2.enable_torque(self.base_motors)

    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        wheel_vel = self.bus2.sync_read("Present_Velocity", self.base_motors)
        return self._wheel_raw_to_body(
            wheel_vel["base_left_wheel"],
            wheel_vel["base_back_wheel"],
            wheel_vel["base_right_wheel"],
        )

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        x = float(action.get("x.vel", 0.0))
        y = float(action.get("y.vel", 0.0))
        theta = float(action.get("theta.vel", 0.0))

        self.bus2.sync_write("Goal_Velocity", self._body_to_wheel_raw(x, y, theta))

        return {"x.vel": x, "y.vel": y, "theta.vel": theta}

    def disconnect(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        self.stop_base()
        self.bus2.disconnect(self.config.disable_torque_on_disconnect)
        logger.info(f"{self} disconnected.")
