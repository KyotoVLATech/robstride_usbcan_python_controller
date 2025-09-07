import enum
import math
import struct
import time
from typing import Optional, Union

import serial

# --- 制御の基本要素を定義 ---


class CommandType(enum.Enum):
    """モーターへ送信するコマンドの種類"""

    GET_DEVICE_ID = 0x00  # 疎通確認用
    ENABLE = 0x03  # モーター有効化
    READ_PARAM = 0x11  # パラメータ読み取り
    WRITE_PARAM = 0x12  # パラメータ書き込み


class MotorStatus(enum.Enum):
    """モーターからの応答に含まれる状態"""

    RESET = 0
    CALIBRATION = 1
    RUN = 2
    UNKNOWN = 99


class RunMode(enum.Enum):
    """モーターの運転モード"""

    OPERATION = 0
    POSITION_PP = 1
    VELOCITY = 2
    CURRENT = 3
    POSITION_CSP = 5


# --- パラメータのインデックスを定数として定義 ---
PARAM_RUN_MODE = 0x7005
PARAM_VEL_MAX = 0x7024
PARAM_ACC_SET = 0x7025
PARAM_LOC_REF = 0x7016


class RobStridePP:
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
        """ATコマンドのフレームを生成します。"""
        if command_type in [CommandType.GET_DEVICE_ID, CommandType.ENABLE]:
            data_area2 = self.host_id

        can_id_29bit = (command_type.value << 24) | (data_area2 << 8) | self.motor_id
        encoded_id_32bit = (can_id_29bit << 3) | 0b100
        header = b'\x41\x54'
        encoded_id_bytes = encoded_id_32bit.to_bytes(4, 'big')
        extended_frame_flag = b'\x08'
        tail = b'\x0d\x0a'
        return header + encoded_id_bytes + extended_frame_flag + data_payload + tail

    def _send_and_receive(self, frame: bytes) -> Optional[bytes]:
        """コマンドを送信し、モーターからの応答を1フレーム受信します。"""
        if not self.ser or not self.ser.is_open:
            print("❌ エラー: シリアルポートが開かれていません。")
            return None
        self.ser.reset_input_buffer()
        self.ser.write(frame)
        response = self.ser.read_until(b'\x0d\x0a', size=17)
        if response and response.startswith(b'AT') and response.endswith(b'\r\n'):
            return response
        elif not response:
            print("❌ 検証失敗: モーターから時間内に応答がありませんでした。")
        else:
            print(f"❌ 検証失敗: 不正な形式の応答を受信しました -> {response.hex(' ')}")
        return None

    def _read_parameter(self, index: int) -> Optional[bytes]:
        """指定したインデックスのパラメータをモーターから読み取ります。"""
        # 読み取りコマンドのデータペイロードを作成 (indexをリトルエンディアンで指定)
        payload = struct.pack('<H', index) + b'\x00' * 6
        frame = self._create_frame(CommandType.READ_PARAM, self.host_id, payload)
        response = self._send_and_receive(frame)
        if response:
            # 応答データペイロードの4バイト目からがパラメータの値
            return response[11:15]
        return None

    def _write_parameter(self, index: int, value: Union[int, float]) -> Optional[bytes]:
        """指定したインデックスにパラメータを書き込み、モーターからのフィードバック応答を返します。"""
        if isinstance(value, int):
            # 整数値の場合 (run_mode用)
            payload = struct.pack('<H', index) + b'\x00\x00' + struct.pack('<I', value)
        elif isinstance(value, float):
            # 浮動小数点数の場合 (速度、加速度、位置用)
            payload = struct.pack('<H', index) + b'\x00\x00' + struct.pack('<f', value)
        else:
            return None

        frame = self._create_frame(CommandType.WRITE_PARAM, self.host_id, payload)
        return self._send_and_receive(frame)

    # --- 公開API ---

    def connect(self) -> bool:
        """シリアルポートに接続し、疎通確認を行います。"""
        print("--- ステップ1: 接続開始 ---")
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1.0)
            print("✅ ポートを開きました。")
        except serial.SerialException as e:
            print(f"❌ ポートを開けませんでした: {e}")
            return False

        frame = self._create_frame(CommandType.GET_DEVICE_ID)
        if self._send_and_receive(frame):
            print("✅ 接続成功: モーターから正常な応答がありました。")
            return True
        else:
            self.disconnect()
            return False

    def enable(self) -> bool:
        """モーターを有効化し、RUN状態になったか検証します。"""
        print("\n--- モーター有効化 ---")
        frame = self._create_frame(CommandType.ENABLE)
        response = self._send_and_receive(frame)
        if not response:
            return False

        # 応答からステータスを解析
        can_id_29bit = int.from_bytes(response[2:6], 'big') >> 3
        status_val = (can_id_29bit >> 22) & 0b11
        status = (
            MotorStatus(status_val)
            if status_val in [m.value for m in MotorStatus]
            else MotorStatus.UNKNOWN
        )

        if status == MotorStatus.RUN:
            print("✅ 有効化成功: モーターはRUN状態に移行しました。")
            return True
        else:
            print(
                f"❌ 有効化失敗: モーターステータスが不正です (現在: {status.name})。"
            )
            return False

    def set_mode_pp(self) -> bool:
        """モーターをPPモードに設定し、正しく設定されたか検証します。"""
        print("\n--- PPモード設定 ---")
        # 書き込み
        self._write_parameter(PARAM_RUN_MODE, RunMode.POSITION_PP.value)

        # 読み取りと検証
        time.sleep(0.1)  # 設定反映待ち
        read_data = self._read_parameter(PARAM_RUN_MODE)
        if read_data:
            current_mode = int.from_bytes(read_data[0:1], 'little')
            if current_mode == RunMode.POSITION_PP.value:
                print("✅ モード設定成功: run_modeが正しく1に設定されました。")
                return True
            else:
                print(
                    f"❌ モード設定失敗: run_modeが予期せぬ値です (現在値: {current_mode})。"
                )
        return False

    def set_pp_velocity(self, velocity: float) -> bool:
        """PPモードの最大速度を設定し、検証します。"""
        print(f"\n--- PPモード最大速度設定 ({velocity} rad/s) ---")
        self._write_parameter(PARAM_VEL_MAX, velocity)

        time.sleep(0.1)
        read_data = self._read_parameter(PARAM_VEL_MAX)
        if read_data:
            current_vel = struct.unpack('<f', read_data)[0]
            # 浮動小数点数の比較は誤差を許容
            if math.isclose(current_vel, velocity, rel_tol=1e-6):
                print(
                    f"✅ 速度設定成功: vel_maxが正しく設定されました (実測値: {current_vel:.2f})。"
                )
                return True
            else:
                print(
                    f"❌ 速度設定失敗: vel_maxが予期せぬ値です (実測値: {current_vel:.2f})。"
                )
        return False

    def set_pp_acceleration(self, acceleration: float) -> bool:
        """PPモードの加速度を設定し、検証します。"""
        print(f"\n--- PPモード加速度設定 ({acceleration} rad/s^2) ---")
        self._write_parameter(PARAM_ACC_SET, acceleration)

        time.sleep(0.1)
        read_data = self._read_parameter(PARAM_ACC_SET)
        if read_data:
            current_acc = struct.unpack('<f', read_data)[0]
            if math.isclose(current_acc, acceleration, rel_tol=1e-6):
                print(
                    f"✅ 加速度設定成功: acc_setが正しく設定されました (実測値: {current_acc:.2f})。"
                )
                return True
            else:
                print(
                    f"❌ 加速度設定失敗: acc_setが予期せぬ値です (実測値: {current_acc:.2f})。"
                )
        return False

    def set_target_position(self, position_rad: float):
        """目標位置を送信します。"""
        print(f"\n--- 目標位置指令 ({position_rad:.2f} rad) ---")
        self._write_parameter(PARAM_LOC_REF, position_rad)
        print("✅ 位置指令を送信しました。")

    def disconnect(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("\nポートを閉じました。")


# --- このライブラリの使用例 ---
if __name__ == '__main__':
    SERIAL_PORT = "COM12"
    MOTOR_ID = 127

    motor = RobStridePP(port=SERIAL_PORT, motor_id=MOTOR_ID)

    if motor.connect():
        # --- マニュアル通りの厳密な手順を実行 ---
        # 1. Disable状態でモードを設定
        if motor.set_mode_pp():
            # 2. モーターを有効化
            if motor.enable():
                time.sleep(0.5)  # 有効化後の安定待ち
                # 3. Enable状態でパラメータを設定
                if motor.set_pp_velocity(5.0):
                    if motor.set_pp_acceleration(10.0):
                        # 4. 位置指令を送信
                        motor.set_target_position(math.pi / 2)  # 90度
                        time.sleep(3)  # 移動を待つ

                        motor.set_target_position(0.0)  # 0度
                        time.sleep(3)

    motor.disconnect()
