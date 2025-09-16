import logging
import math
import struct
import time
from dataclasses import dataclass
from logging import Formatter, StreamHandler, getLogger
from typing import Any, Dict, List, Optional, Union

import serial

from src.constants import CommandType, MotorStatus, ParameterIndex, RunMode

# Improved logger configuration
logger = getLogger(__name__)
logger.setLevel(logging.DEBUG)
handler_format = Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
stream_handler = StreamHandler()
stream_handler.setLevel(logging.INFO)
stream_handler.setFormatter(handler_format)
logger.addHandler(stream_handler)


@dataclass
class RobStride:
    id: int
    max_position: float  # radians(0 to 2*pi)
    min_position: float  # radians(0 to 2*pi)
    max_velocity: float  # radians/sec
    enc_offset: float  # encoder offset radians


class RobStrideController:
    def __init__(
        self,
        motors: List[RobStride],
        port: str,
        baudrate: int = 921600,
        host_id: int = 253,
    ):
        self.motors = {motor.id: motor for motor in motors}
        self.port = port
        self.baudrate = baudrate
        self.host_id = host_id
        self.ser: Optional[serial.Serial] = None

    def _create_frame(
        self,
        command_type: CommandType,
        motor_id: int,
        data_area2: int = 0,
        data_payload: bytes = b'\x00' * 8,
    ) -> bytes:
        if command_type in [
            CommandType.GET_DEVICE_ID,
            CommandType.ENABLE,
            CommandType.DISABLE,
        ]:
            data_area2 = self.host_id

        can_id_29bit = (command_type.value << 24) | (data_area2 << 8) | motor_id
        encoded_id_32bit = (can_id_29bit << 3) | 0b100
        header = b'\x41\x54'
        encoded_id_bytes = encoded_id_32bit.to_bytes(4, 'big')
        extended_frame_flag = b'\x08'
        tail = b'\x0d\x0a'
        byte = header + encoded_id_bytes + extended_frame_flag + data_payload + tail
        assert isinstance(byte, bytes) and len(byte) == 17, "Frame must be 17 bytes"
        return byte

    def _send_and_receive(self, frame: bytes) -> Optional[bytes]:
        if not self.ser or not self.ser.is_open:
            logger.error("Serial port is not open")
            return None
        self.ser.reset_input_buffer()
        self.ser.write(frame)
        logger.debug(f"Sent frame: {frame.hex(' ')}")
        response = self.ser.read_until(b'\x0d\x0a', size=17)
        if response and response.startswith(b'AT') and response.endswith(b'\r\n'):
            logger.debug(f"Received valid response: {response.hex(' ')}")
            byte = response
            assert isinstance(byte, bytes) and len(byte) == 17
            return byte
        elif not response:
            logger.error("No response received from motor within timeout")
        else:
            logger.error(f"Invalid response format received: {response.hex(' ')}")
        return None

    def _read_parameter(self, motor_id: int, index: int) -> Optional[bytes]:
        payload = struct.pack('<H', index) + b'\x00' * 6
        frame = self._create_frame(
            CommandType.READ_PARAM, motor_id, self.host_id, payload
        )
        response = self._send_and_receive(frame)
        if response:
            return response[11:15]
        return None

    def _write_parameter(
        self, motor_id: int, index: int, value: Union[int, float]
    ) -> Optional[bytes]:
        if isinstance(value, int):
            # For RunMode, which is a uint8, pack as a 4-byte integer
            payload = struct.pack('<H', index) + b'\x00\x00' + struct.pack('<I', value)
        elif isinstance(value, float):
            payload = struct.pack('<H', index) + b'\x00\x00' + struct.pack('<f', value)
        else:
            return None

        frame = self._create_frame(
            CommandType.WRITE_PARAM, motor_id, self.host_id, payload
        )
        return self._send_and_receive(frame)

    def connect(self) -> bool:
        logger.info("Initiating connection to motors")
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1.0)
            logger.info(f"Serial port {self.port} opened successfully")
        except serial.SerialException as e:
            logger.error(f"Failed to open serial port {self.port}: {e}")
            return False

        # 全てのモーターに対して接続確認
        connected_motors = []
        for motor_id in self.motors.keys():
            frame = self._create_frame(CommandType.GET_DEVICE_ID, motor_id)
            if self._send_and_receive(frame):
                logger.info(
                    f"Connection established successfully with motor ID {motor_id}"
                )
                connected_motors.append(motor_id)
            else:
                logger.error(f"Failed to establish connection with motor ID {motor_id}")

        if connected_motors:
            logger.info(
                f"Successfully connected to {len(connected_motors)}/{len(self.motors)} motors"
            )
            return True
        else:
            logger.error("Failed to connect to any motors")
            self.disconnect()
            return False

    def enable(self, motor_id: int) -> bool:
        if motor_id not in self.motors:
            logger.error(f"Motor ID {motor_id} not found in motor list")
            return False

        logger.info(f"Enabling motor ID {motor_id}")
        frame = self._create_frame(CommandType.ENABLE, motor_id)
        response = self._send_and_receive(frame)
        if not response:
            logger.error(f"Failed to send enable command to motor ID {motor_id}")
            return False

        can_id_29bit = int.from_bytes(response[2:6], 'big') >> 3
        status_val = (can_id_29bit >> 22) & 0b11
        status = (
            MotorStatus(status_val)
            if status_val in [m.value for m in MotorStatus]
            else MotorStatus.UNKNOWN
        )

        if status == MotorStatus.RUN:
            logger.info(
                f"Motor ID {motor_id} enabled successfully and entered RUN state"
            )
            return True
        else:
            logger.error(
                f"Motor ID {motor_id} enable failed: Invalid status {status.name}"
            )
            return False

    def enable_all(self) -> Dict[int, bool]:
        """全てのモーターを有効化します。"""
        results = {}
        for motor_id in self.motors.keys():
            results[motor_id] = self.enable(motor_id)
        return results

    def disable(self, motor_id: int) -> None:
        """指定されたモーターを無効化（運転停止）します。"""
        if motor_id not in self.motors:
            logger.error(f"Motor ID {motor_id} not found in motor list")
            return

        logger.info(f"Disabling motor ID {motor_id}")
        frame = self._create_frame(CommandType.DISABLE, motor_id)
        self._send_and_receive(frame)
        logger.info(f"Disable command sent successfully to motor ID {motor_id}")

    def disable_all(self) -> None:
        """全てのモーターを無効化します。"""
        for motor_id in self.motors.keys():
            self.disable(motor_id)

    def _set_run_mode(self, motor_id: int, mode: RunMode) -> bool:
        if motor_id not in self.motors:
            logger.error(f"Motor ID {motor_id} not found in motor list")
            return False

        logger.info(f"Setting motor ID {motor_id} to {mode.name} mode")
        self._write_parameter(motor_id, ParameterIndex.RUN_MODE.value, mode.value)

        time.sleep(0.1)
        read_data = self._read_parameter(motor_id, ParameterIndex.RUN_MODE.value)
        if read_data:
            current_mode = int.from_bytes(read_data[0:1], 'little')
            if current_mode == mode.value:
                logger.info(f"Motor ID {motor_id} {mode.name} mode set successfully")
                return True
            else:
                logger.error(
                    f"Motor ID {motor_id} {mode.name} mode setting failed: Unexpected run_mode value {current_mode}"
                )
        else:
            logger.error(f"Failed to read run_mode parameter for motor ID {motor_id}")
        return False

    def _set_float_parameter(
        self,
        motor_id: int,
        param_index: ParameterIndex,
        value: float,
        name: str,
        unit: str,
    ) -> bool:
        if motor_id not in self.motors:
            logger.error(f"Motor ID {motor_id} not found in motor list")
            return False

        logger.info(f"Setting {name} to {value} {unit} for motor ID {motor_id}")
        self._write_parameter(motor_id, param_index.value, value)

        time.sleep(0.1)
        read_data = self._read_parameter(motor_id, param_index.value)
        if read_data:
            current_val = struct.unpack('<f', read_data)[0]
            if math.isclose(current_val, value, rel_tol=1e-6):
                logger.info(
                    f"Motor ID {motor_id} {name} set successfully to {current_val:.2f} {unit}"
                )
                return True
            else:
                logger.error(
                    f"Motor ID {motor_id} {name} setting failed: Expected {value:.2f}, got {current_val:.2f} {unit}"
                )
        else:
            logger.error(f"Failed to read {name} parameter for motor ID {motor_id}")
        return False

    # --- PP (Profile Position) Mode Methods ---
    def set_mode_pp(self, motor_id: int) -> bool:
        return self._set_run_mode(motor_id, RunMode.POSITION_PP)

    def set_pp_velocity(self, motor_id: int, velocity: float) -> bool:
        return self._set_float_parameter(
            motor_id, ParameterIndex.VEL_MAX, velocity, "PP velocity", "rad/s"
        )

    def set_pp_acceleration(self, motor_id: int, acceleration: float) -> bool:
        return self._set_float_parameter(
            motor_id, ParameterIndex.ACC_SET, acceleration, "PP acceleration", "rad/s^2"
        )

    def set_target_position(self, motor_id: int, position_rad: float) -> None:
        if motor_id not in self.motors:
            logger.error(f"Motor ID {motor_id} not found in motor list")
            return

        logger.info(
            f"Setting target position to {position_rad:.2f} rad for motor ID {motor_id}"
        )
        self._write_parameter(motor_id, ParameterIndex.LOC_REF.value, position_rad)
        logger.info(f"Target position command sent successfully to motor ID {motor_id}")

    # --- Velocity Mode Methods ---
    def set_mode_velocity(self, motor_id: int) -> bool:
        return self._set_run_mode(motor_id, RunMode.VELOCITY)

    def set_velocity_limit_cur(self, motor_id: int, current: float) -> bool:
        return self._set_float_parameter(
            motor_id, ParameterIndex.LIMIT_CUR, current, "Velocity current limit", "A"
        )

    def set_velocity_acceleration(self, motor_id: int, acceleration: float) -> bool:
        return self._set_float_parameter(
            motor_id,
            ParameterIndex.ACC_RAD,
            acceleration,
            "Velocity acceleration",
            "rad/s^2",
        )

    def set_target_velocity(self, motor_id: int, velocity: float) -> None:
        if motor_id not in self.motors:
            logger.error(f"Motor ID {motor_id} not found in motor list")
            return

        logger.info(
            f"Setting target velocity to {velocity:.2f} rad/s for motor ID {motor_id}"
        )
        self._write_parameter(motor_id, ParameterIndex.SPD_REF.value, velocity)
        logger.info(f"Target velocity command sent successfully to motor ID {motor_id}")

    # --- Current Mode Methods ---
    def set_mode_current(self, motor_id: int) -> bool:
        return self._set_run_mode(motor_id, RunMode.CURRENT)

    def set_target_current(self, motor_id: int, current: float) -> None:
        if motor_id not in self.motors:
            logger.error(f"Motor ID {motor_id} not found in motor list")
            return

        logger.info(
            f"Setting target current to {current:.2f} A for motor ID {motor_id}"
        )
        self._write_parameter(motor_id, ParameterIndex.IQ_REF.value, current)
        logger.info(f"Target current command sent successfully to motor ID {motor_id}")

    # --- CSP (Cyclic Synchronous Position) Mode Methods ---
    def set_mode_csp(self, motor_id: int) -> bool:
        return self._set_run_mode(motor_id, RunMode.POSITION_CSP)

    def set_csp_velocity_limit(self, motor_id: int, velocity: float) -> bool:
        return self._set_float_parameter(
            motor_id, ParameterIndex.LIMIT_SPD, velocity, "CSP velocity limit", "rad/s"
        )

    def disconnect(self) -> None:
        if self.ser and self.ser.is_open:
            self.ser.close()
            logger.info("Serial port closed")

    def __enter__(self) -> 'RobStrideController':
        """with構文の開始時に接続を行います。"""
        if self.connect():
            return self
        else:
            raise IOError("Failed to establish connection with motors")

    def __exit__(self, exc_type: Any, exc_val: Any, exc_tb: Any) -> None:
        """with構文の終了時に、安全にモーターを停止し、切断します。"""
        if self.ser and self.ser.is_open:
            # 念のため全モーターの目標値を0にしてから停止
            for motor_id in self.motors.keys():
                self.set_target_velocity(motor_id, 0.0)
                self.set_target_current(motor_id, 0.0)
            time.sleep(0.1)
            self.disable_all()
        self.disconnect()
