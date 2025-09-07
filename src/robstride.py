import logging
import math
import struct
import time
from logging import Formatter, StreamHandler, getLogger
from typing import Any, Optional, Union

import serial

from constants import CommandType, MotorStatus, ParameterIndex, RunMode

# Improved logger configuration
logger = getLogger(__name__)
logger.setLevel(logging.DEBUG)
handler_format = Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
stream_handler = StreamHandler()
stream_handler.setLevel(logging.INFO)
stream_handler.setFormatter(handler_format)
logger.addHandler(stream_handler)


class RobStride:
    def __init__(
        self, port: str, baudrate: int = 921600, motor_id: int = 127, host_id: int = 253
    ):
        self.port = port
        self.baudrate = baudrate
        self.motor_id = motor_id
        self.host_id = host_id
        self.ser: Optional[serial.Serial] = None

    def _create_frame(
        self,
        command_type: CommandType,
        data_area2: int = 0,
        data_payload: bytes = b'\x00' * 8,
    ) -> bytes:
        if command_type in [
            CommandType.GET_DEVICE_ID,
            CommandType.ENABLE,
            CommandType.DISABLE,
        ]:
            data_area2 = self.host_id

        can_id_29bit = (command_type.value << 24) | (data_area2 << 8) | self.motor_id
        encoded_id_32bit = (can_id_29bit << 3) | 0b100
        header = b'\x41\x54'
        encoded_id_bytes = encoded_id_32bit.to_bytes(4, 'big')
        extended_frame_flag = b'\x08'
        tail = b'\x0d\x0a'
        return header + encoded_id_bytes + extended_frame_flag + data_payload + tail

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
            return response
        elif not response:
            logger.error("No response received from motor within timeout")
        else:
            logger.error(f"Invalid response format received: {response.hex(' ')}")
        return None

    def _read_parameter(self, index: int) -> Optional[bytes]:
        payload = struct.pack('<H', index) + b'\x00' * 6
        frame = self._create_frame(CommandType.READ_PARAM, self.host_id, payload)
        response = self._send_and_receive(frame)
        if response:
            return response[11:15]
        return None

    def _write_parameter(self, index: int, value: Union[int, float]) -> Optional[bytes]:
        if isinstance(value, int):
            # For RunMode, which is a uint8, pack as a 4-byte integer
            payload = struct.pack('<H', index) + b'\x00\x00' + struct.pack('<I', value)
        elif isinstance(value, float):
            payload = struct.pack('<H', index) + b'\x00\x00' + struct.pack('<f', value)
        else:
            return None

        frame = self._create_frame(CommandType.WRITE_PARAM, self.host_id, payload)
        return self._send_and_receive(frame)

    def connect(self) -> bool:
        logger.info("Initiating connection to motor")
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1.0)
            logger.info(f"Serial port {self.port} opened successfully")
        except serial.SerialException as e:
            logger.error(f"Failed to open serial port {self.port}: {e}")
            return False

        frame = self._create_frame(CommandType.GET_DEVICE_ID)
        if self._send_and_receive(frame):
            logger.info(
                f"Connection established successfully with motor ID {self.motor_id}"
            )
            return True
        else:
            logger.error("Failed to establish connection with motor")
            self.disconnect()
            return False

    def enable(self) -> bool:
        logger.info("Enabling motor")
        frame = self._create_frame(CommandType.ENABLE)
        response = self._send_and_receive(frame)
        if not response:
            logger.error("Failed to send enable command")
            return False

        can_id_29bit = int.from_bytes(response[2:6], 'big') >> 3
        status_val = (can_id_29bit >> 22) & 0b11
        status = (
            MotorStatus(status_val)
            if status_val in [m.value for m in MotorStatus]
            else MotorStatus.UNKNOWN
        )

        if status == MotorStatus.RUN:
            logger.info("Motor enabled successfully and entered RUN state")
            return True
        else:
            logger.error(f"Motor enable failed: Invalid status {status.name}")
            return False

    def disable(self) -> None:
        """モーターを無効化（運転停止）します。"""
        logger.info("Disabling motor")
        frame = self._create_frame(CommandType.DISABLE)
        self._send_and_receive(frame)
        logger.info("Disable command sent successfully")

    def _set_run_mode(self, mode: RunMode) -> bool:
        logger.info(f"Setting motor to {mode.name} mode")
        self._write_parameter(ParameterIndex.RUN_MODE.value, mode.value)

        time.sleep(0.1)
        read_data = self._read_parameter(ParameterIndex.RUN_MODE.value)
        if read_data:
            current_mode = int.from_bytes(read_data[0:1], 'little')
            if current_mode == mode.value:
                logger.info(f"{mode.name} mode set successfully")
                return True
            else:
                logger.error(
                    f"{mode.name} mode setting failed: Unexpected run_mode value {current_mode}"
                )
        else:
            logger.error("Failed to read run_mode parameter")
        return False

    def _set_float_parameter(
        self, param_index: ParameterIndex, value: float, name: str, unit: str
    ) -> bool:
        logger.info(f"Setting {name} to {value} {unit}")
        self._write_parameter(param_index.value, value)

        time.sleep(0.1)
        read_data = self._read_parameter(param_index.value)
        if read_data:
            current_val = struct.unpack('<f', read_data)[0]
            if math.isclose(current_val, value, rel_tol=1e-6):
                logger.info(f"{name} set successfully to {current_val:.2f} {unit}")
                return True
            else:
                logger.error(
                    f"{name} setting failed: Expected {value:.2f}, got {current_val:.2f} {unit}"
                )
        else:
            logger.error(f"Failed to read {name} parameter")
        return False

    # --- PP (Profile Position) Mode Methods ---
    def set_mode_pp(self) -> bool:
        return self._set_run_mode(RunMode.POSITION_PP)

    def set_pp_velocity(self, velocity: float) -> bool:
        return self._set_float_parameter(
            ParameterIndex.VEL_MAX, velocity, "PP velocity", "rad/s"
        )

    def set_pp_acceleration(self, acceleration: float) -> bool:
        return self._set_float_parameter(
            ParameterIndex.ACC_SET, acceleration, "PP acceleration", "rad/s^2"
        )

    def set_target_position(self, position_rad: float) -> None:
        logger.info(f"Setting target position to {position_rad:.2f} rad")
        self._write_parameter(ParameterIndex.LOC_REF.value, position_rad)
        logger.info("Target position command sent successfully")

    # --- Velocity Mode Methods ---
    def set_mode_velocity(self) -> bool:
        return self._set_run_mode(RunMode.VELOCITY)

    def set_velocity_limit_cur(self, current: float) -> bool:
        return self._set_float_parameter(
            ParameterIndex.LIMIT_CUR, current, "Velocity current limit", "A"
        )

    def set_velocity_acceleration(self, acceleration: float) -> bool:
        return self._set_float_parameter(
            ParameterIndex.ACC_RAD, acceleration, "Velocity acceleration", "rad/s^2"
        )

    def set_target_velocity(self, velocity: float) -> None:
        logger.info(f"Setting target velocity to {velocity:.2f} rad/s")
        self._write_parameter(ParameterIndex.SPD_REF.value, velocity)
        logger.info("Target velocity command sent successfully")

    # --- Current Mode Methods ---
    def set_mode_current(self) -> bool:
        return self._set_run_mode(RunMode.CURRENT)

    def set_target_current(self, current: float) -> None:
        logger.info(f"Setting target current to {current:.2f} A")
        self._write_parameter(ParameterIndex.IQ_REF.value, current)
        logger.info("Target current command sent successfully")

    # --- CSP (Cyclic Synchronous Position) Mode Methods ---
    def set_mode_csp(self) -> bool:
        return self._set_run_mode(RunMode.POSITION_CSP)

    def set_csp_velocity_limit(self, velocity: float) -> bool:
        return self._set_float_parameter(
            ParameterIndex.LIMIT_SPD, velocity, "CSP velocity limit", "rad/s"
        )

    def disconnect(self) -> None:
        if self.ser and self.ser.is_open:
            self.ser.close()
            logger.info("Serial port closed")

    def __enter__(self) -> 'RobStride':
        """with構文の開始時に接続を行います。"""
        if self.connect():
            return self
        else:
            raise IOError("Failed to establish connection with motor")

    def __exit__(self, exc_type: Any, exc_val: Any, exc_tb: Any) -> None:
        """with構文の終了時に、安全にモーターを停止し、切断します。"""
        if self.ser and self.ser.is_open:
            # 念のため目標値を0にしてから停止
            self.set_target_velocity(0.0)
            self.set_target_current(0.0)
            time.sleep(0.1)
            self.disable()
        self.disconnect()
