import asyncio
import logging
import math
import struct
from asyncio import Lock
from dataclasses import dataclass
from logging import Formatter, StreamHandler, getLogger
from typing import Any, Optional, Union

import serial_asyncio

from .constants import CommandType, MotorStatus, ParameterIndex, RunMode

# Improved logger configuration
logger = getLogger(__name__)
logger.setLevel(logging.DEBUG)
handler_format = Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
stream_handler = StreamHandler()
stream_handler.setLevel(logging.INFO)
stream_handler.setFormatter(handler_format)
logger.addHandler(stream_handler)


@dataclass
class RobStrideLimits:
    """RobStrideモーターの制限パラメータを管理するクラス"""

    # PP (Profile Position) Mode limits
    pp_vel_max: Optional[float] = None  # 最大速度 (rad/s)
    pp_acc_set: Optional[float] = None  # 加速度 (rad/s^2)
    pp_limit_cur: Optional[float] = None  # 電流制限 (A)

    # Velocity Mode limits
    velocity_limit_cur: Optional[float] = None  # 電流制限 (A)
    velocity_acc_rad: Optional[float] = None  # 加速度 (rad/s^2)

    # CSP (Cyclic Synchronous Position) Mode limits
    csp_limit_spd: Optional[float] = None  # 速度制限 (rad/s)
    csp_limit_cur: Optional[float] = None  # 電流制限 (A)


@dataclass
class RobStride:
    id: int
    offset: float  # in radians
    limits: Optional[RobStrideLimits] = None
    _is_enabled: bool = False
    _current_mode: Optional[RunMode] = None

    def is_enabled(self) -> bool:
        """モーターの有効状態を返す"""
        return self._is_enabled

    def get_current_mode(self) -> Optional[RunMode]:
        """現在の制御モードを返す"""
        return self._current_mode

    def _set_enabled(self, enabled: bool) -> None:
        """モーターの有効状態を設定（内部使用）"""
        self._is_enabled = enabled

    def _set_mode(self, mode: Optional[RunMode]) -> None:
        """現在の制御モードを設定（内部使用）"""
        self._current_mode = mode


class RobStrideController:
    def __init__(
        self,
        port: str,
        motors: list[RobStride],
        baudrate: int = 921600,
        host_id: int = 253,
    ):
        self.port = port
        self.baudrate = baudrate
        self.motors = {motor.id: motor for motor in motors}
        self.host_id = host_id
        self.reader: Optional[asyncio.StreamReader] = None
        self.writer: Optional[asyncio.StreamWriter] = None
        self.lock = Lock()

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

    async def _send_and_receive(self, frame: bytes) -> Optional[bytes]:
        # ★ このロックが単一バスの混線を防ぐ
        async with self.lock:
            if not self.writer or self.writer.is_closing():
                logger.error("Serial connection is not open")
                return None

            try:
                # Clear any pending data in the reader
                # Note: StreamReader doesn't have reset_input_buffer,
                # but we can read and discard any pending data
                while True:
                    try:
                        # Try to read with a very short timeout
                        pending = await asyncio.wait_for(
                            self.reader.read(1024), timeout=0.001
                        )
                        if not pending:
                            break
                    except asyncio.TimeoutError:
                        break

                # Send the frame
                self.writer.write(frame)
                await self.writer.drain()
                logger.debug(f"Sent frame: {frame.hex(' ')}")

                # Read response with timeout
                response = await asyncio.wait_for(
                    self.reader.readuntil(b'\x0d\x0a'), timeout=1.0
                )

            except asyncio.TimeoutError:
                logger.error("No response received from motor within timeout")
                return None
            except Exception as e:
                logger.error(f"Error during serial I/O: {e}")
                return None

            if response and response.startswith(b'AT') and response.endswith(b'\r\n'):
                logger.debug(f"Received valid response: {response.hex(' ')}")
                if len(response) == 17:
                    return response
                else:
                    logger.error(
                        f"Invalid response length: expected 17 bytes, got {len(response)}"
                    )
            else:
                logger.error(
                    f"Invalid response format received: {response.hex(' ') if response else 'None'}"
                )
            return None

    async def _read_parameter(self, motor_id: int, index: int) -> Optional[bytes]:
        payload = struct.pack('<H', index) + b'\x00' * 6
        frame = self._create_frame(
            CommandType.READ_PARAM, motor_id, self.host_id, payload
        )
        response = await self._send_and_receive(frame)
        if response:
            return response[11:15]
        return None

    async def _write_parameter(
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
        return await self._send_and_receive(frame)

    async def connect(self) -> bool:
        logger.info("Initiating connection to motors")
        try:
            # Create serial connection with asyncio
            coro = serial_asyncio.open_serial_connection(
                url=self.port, baudrate=self.baudrate, timeout=1.0
            )
            self.reader, self.writer = await coro
            logger.info(f"Serial port {self.port} opened successfully")
        except Exception as e:
            logger.error(f"Failed to open serial port {self.port}: {e}")
            return False

        # Check connection to all motors
        all_connected = True
        for motor_id in self.motors.keys():
            frame = self._create_frame(CommandType.GET_DEVICE_ID, motor_id)
            if await self._send_and_receive(frame):
                logger.info(
                    f"Connection established successfully with motor ID {motor_id}"
                )
            else:
                logger.error(f"Failed to establish connection with motor ID {motor_id}")
                all_connected = False

        if all_connected:
            logger.info("All motors connected successfully")
            return True
        else:
            logger.error("Failed to connect to some motors")
            await self.disconnect()
            return False

    async def enable(self, motor_id: int) -> bool:
        if motor_id not in self.motors:
            logger.error(f"Motor ID {motor_id} not found in motor list")
            return False

        logger.info(f"Enabling motor {motor_id}")
        frame = self._create_frame(CommandType.ENABLE, motor_id)
        response = await self._send_and_receive(frame)
        if not response:
            logger.error(f"Failed to send enable command to motor {motor_id}")
            return False

        can_id_29bit = int.from_bytes(response[2:6], 'big') >> 3
        status_val = (can_id_29bit >> 22) & 0b11
        status = (
            MotorStatus(status_val)
            if status_val in [m.value for m in MotorStatus]
            else MotorStatus.UNKNOWN
        )

        if status == MotorStatus.RUN:
            logger.info(f"Motor {motor_id} enabled successfully and entered RUN state")
            self.motors[motor_id]._set_enabled(True)
            return True
        else:
            logger.error(
                f"Motor {motor_id} enable failed: Invalid status {status.name}"
            )
            return False

    async def disable(self, motor_id: int) -> None:
        """指定されたモーターを無効化（運転停止）します。"""
        if motor_id not in self.motors:
            logger.error(f"Motor ID {motor_id} not found in motor list")
            return

        logger.info(f"Disabling motor {motor_id}")
        frame = self._create_frame(CommandType.DISABLE, motor_id)
        await self._send_and_receive(frame)
        self.motors[motor_id]._set_enabled(False)
        self.motors[motor_id]._set_mode(None)
        logger.info(f"Disable command sent successfully to motor {motor_id}")

    def _check_motor_enabled(self, motor_id: int) -> bool:
        """モーターが有効かどうかをチェック"""
        if motor_id not in self.motors:
            logger.error(f"Motor ID {motor_id} not found in motor list")
            return False

        if not self.motors[motor_id].is_enabled():
            logger.error(
                f"Motor {motor_id} is not enabled. Please enable the motor first."
            )
            return False

        return True

    def _check_motor_mode(self, motor_id: int, required_mode: RunMode) -> bool:
        """モーターが指定されたモードかどうかをチェック"""
        if not self._check_motor_enabled(motor_id):
            return False

        current_mode = self.motors[motor_id].get_current_mode()
        if current_mode != required_mode:
            logger.error(
                f"Motor {motor_id} is not in {required_mode.name} mode. Current mode: {current_mode.name if current_mode else 'None'}"
            )
            return False

        return True

    async def _set_run_mode(self, motor_id: int, mode: RunMode) -> bool:
        logger.info(f"Setting motor {motor_id} to {mode.name} mode")
        await self._write_parameter(motor_id, ParameterIndex.RUN_MODE.value, mode.value)

        await asyncio.sleep(0.1)
        read_data = await self._read_parameter(motor_id, ParameterIndex.RUN_MODE.value)
        if read_data:
            current_mode = int.from_bytes(read_data[0:1], 'little')
            if current_mode == mode.value:
                logger.info(f"Motor {motor_id} {mode.name} mode set successfully")
                self.motors[motor_id]._set_mode(mode)
                return True
            else:
                logger.error(
                    f"Motor {motor_id} {mode.name} mode setting failed: Unexpected run_mode value {current_mode}"
                )
        else:
            logger.error(f"Failed to read run_mode parameter for motor {motor_id}")
        return False

    async def _set_float_parameter(
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

        logger.info(f"Setting {name} to {value} {unit} for motor {motor_id}")
        await self._write_parameter(motor_id, param_index.value, value)

        await asyncio.sleep(0.1)
        read_data = await self._read_parameter(motor_id, param_index.value)
        if read_data:
            current_val = struct.unpack('<f', read_data)[0]
            if math.isclose(current_val, value, rel_tol=1e-6):
                logger.info(
                    f"Motor {motor_id} {name} set successfully to {current_val:.2f} {unit}"
                )
                return True
            else:
                logger.error(
                    f"Motor {motor_id} {name} setting failed: Expected {value:.2f}, got {current_val:.2f} {unit}"
                )
        else:
            logger.error(f"Failed to read {name} parameter for motor {motor_id}")
        return False

    # --- PP (Profile Position) Mode Methods ---
    async def set_mode_pp(self, motor_id: int) -> bool:
        return await self._set_run_mode(motor_id, RunMode.POSITION_PP)

    async def apply_pp_limits(self, motor_id: int) -> bool:
        """PP モードのリミッターを適用"""
        if not self._check_motor_mode(motor_id, RunMode.POSITION_PP):
            return False

        motor = self.motors[motor_id]
        if not motor.limits:
            logger.warning(f"No limits configured for motor {motor_id}")
            return True

        success = True
        limits = motor.limits

        if limits.pp_vel_max is not None:
            success &= await self._set_float_parameter(
                motor_id,
                ParameterIndex.VEL_MAX,
                limits.pp_vel_max,
                "PP velocity",
                "rad/s",
            )

        if limits.pp_acc_set is not None:
            success &= await self._set_float_parameter(
                motor_id,
                ParameterIndex.ACC_SET,
                limits.pp_acc_set,
                "PP acceleration",
                "rad/s^2",
            )

        if limits.pp_limit_cur is not None:
            success &= await self._set_float_parameter(
                motor_id,
                ParameterIndex.LIMIT_CUR,
                limits.pp_limit_cur,
                "PP current limit",
                "A",
            )

        return success

    async def set_target_position(self, motor_id: int, position_rad: float) -> None:
        if motor_id not in self.motors:
            logger.error(f"Motor ID {motor_id} not found in motor list")
            return

        logger.info(
            f"Setting target position to {position_rad:.2f} rad for motor {motor_id}"
        )
        await self._write_parameter(
            motor_id,
            ParameterIndex.LOC_REF.value,
            position_rad + self.motors[motor_id].offset,
        )
        logger.info(f"Target position command sent successfully to motor {motor_id}")

    # --- Velocity Mode Methods ---
    async def set_mode_velocity(self, motor_id: int) -> bool:
        return await self._set_run_mode(motor_id, RunMode.VELOCITY)

    async def apply_velocity_limits(self, motor_id: int) -> bool:
        """Velocity モードのリミッターを適用"""
        if not self._check_motor_mode(motor_id, RunMode.VELOCITY):
            return False

        motor = self.motors[motor_id]
        if not motor.limits:
            logger.warning(f"No limits configured for motor {motor_id}")
            return True

        success = True
        limits = motor.limits

        if limits.velocity_limit_cur is not None:
            success &= await self._set_float_parameter(
                motor_id,
                ParameterIndex.LIMIT_CUR,
                limits.velocity_limit_cur,
                "Velocity current limit",
                "A",
            )

        if limits.velocity_acc_rad is not None:
            success &= await self._set_float_parameter(
                motor_id,
                ParameterIndex.ACC_RAD,
                limits.velocity_acc_rad,
                "Velocity acceleration",
                "rad/s^2",
            )

        return success

    async def set_target_velocity(self, motor_id: int, velocity: float) -> None:
        """速度制御モードで目標速度を設定します。"""
        if motor_id not in self.motors:
            logger.error(f"Motor ID {motor_id} not found in motor list")
            return

        logger.info(
            f"Setting target velocity to {velocity:.2f} rad/s for motor {motor_id}"
        )
        await self._write_parameter(motor_id, ParameterIndex.SPD_REF.value, velocity)
        logger.info(f"Target velocity command sent successfully to motor {motor_id}")

    # --- Current Mode Methods ---
    async def set_mode_current(self, motor_id: int) -> bool:
        return await self._set_run_mode(motor_id, RunMode.CURRENT)

    async def set_target_current(self, motor_id: int, current: float) -> None:
        if motor_id not in self.motors:
            logger.error(f"Motor ID {motor_id} not found in motor list")
            return

        logger.info(f"Setting target current to {current:.2f} A for motor {motor_id}")
        await self._write_parameter(motor_id, ParameterIndex.IQ_REF.value, current)
        logger.info(f"Target current command sent successfully to motor {motor_id}")

    # --- CSP (Cyclic Synchronous Position) Mode Methods ---
    async def set_mode_csp(self, motor_id: int) -> bool:
        return await self._set_run_mode(motor_id, RunMode.POSITION_CSP)

    async def apply_csp_limits(self, motor_id: int) -> bool:
        """CSP モードのリミッターを適用"""
        if not self._check_motor_mode(motor_id, RunMode.POSITION_CSP):
            return False

        motor = self.motors[motor_id]
        if not motor.limits:
            logger.warning(f"No limits configured for motor {motor_id}")
            return True

        success = True
        limits = motor.limits

        if limits.csp_limit_spd is not None:
            success &= await self._set_float_parameter(
                motor_id,
                ParameterIndex.LIMIT_SPD,
                limits.csp_limit_spd,
                "CSP velocity limit",
                "rad/s",
            )

        if limits.csp_limit_cur is not None:
            success &= await self._set_float_parameter(
                motor_id,
                ParameterIndex.LIMIT_CUR,
                limits.csp_limit_cur,
                "CSP current limit",
                "A",
            )

        return success

    async def disconnect(self) -> None:
        if self.writer and not self.writer.is_closing():
            self.writer.close()
            await self.writer.wait_closed()
            logger.info("Serial port closed")

    async def __aenter__(self) -> 'RobStrideController':
        """async with構文の開始時に接続を行います。"""
        if await self.connect():
            return self
        else:
            raise IOError("Failed to establish connection with motor")

    async def __aexit__(self, exc_type: Any, exc_val: Any, exc_tb: Any) -> None:
        """async with構文の終了時に、安全にモーターを停止し、切断します。"""
        if self.writer and not self.writer.is_closing():
            logger.info(f"Safely shutting down motors on {self.port} sequentially...")

            # ★安全な逐次実行（forループ）
            for motor_id in self.motors.keys():
                await self.set_target_velocity(motor_id, 0.0)
                await self.set_target_current(motor_id, 0.0)

            await asyncio.sleep(0.1)

            for motor_id in self.motors.keys():
                await self.disable(motor_id)

        await self.disconnect()

    def __enter__(self) -> 'RobStrideController':
        """with構文の開始時に接続を行います。（非推奨：async withを使用してください）"""
        raise NotImplementedError(
            "Use 'async with' instead of 'with' for RobStrideController"
        )

    def __exit__(self, exc_type: Any, exc_val: Any, exc_tb: Any) -> None:
        """with構文の終了時の処理（非推奨：async withを使用してください）"""
        raise NotImplementedError(
            "Use 'async with' instead of 'with' for RobStrideController"
        )
