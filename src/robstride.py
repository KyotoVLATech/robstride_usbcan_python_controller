import enum
from typing import Optional

import serial

# --- 制御の基本要素を定義 ---


class CommandType(enum.Enum):
    """モーターへ送信するコマンドの種類"""

    GET_DEVICE_ID = 0x00  # 疎通確認用
    ENABLE = 0x03  # モーター有効化


class MotorStatus(enum.Enum):
    """モーターからの応答に含まれる状態"""

    RESET = 0
    CALIBRATION = 1
    RUN = 2
    UNKNOWN = 99


class RobStrideMinimal:
    """
    Robstrideモーターの接続と有効化に特化した最小構成ライブラリ。
    各ステップでモーターからの応答を確実に検証します。
    """

    def __init__(
        self, port: str, baudrate: int = 921600, motor_id: int = 127, host_id: int = 253
    ):
        self.port = port
        self.baudrate = baudrate
        self.motor_id = motor_id
        self.host_id = host_id
        self.ser: Optional[serial.Serial] = None

    def _create_frame(self, command_type: CommandType) -> bytes:
        """ATコマンドのフレームを生成します。"""
        # 29-bit CAN ID の構築
        # Bit 28-24: 通信タイプ, Bit 23-8: ホストID, Bit 7-0: モーターID
        can_id_29bit = (command_type.value << 24) | (self.host_id << 8) | self.motor_id

        # USB-CANアダプタの特殊なエンコーディング
        encoded_id_32bit = (can_id_29bit << 3) | 0b100

        # ATコマンドの各パーツを結合
        header = b'\x41\x54'  # "AT"
        encoded_id_bytes = encoded_id_32bit.to_bytes(4, 'big')
        extended_frame_flag = b'\x08'
        data_payload = b'\x00' * 8  # データ部は0で埋める
        tail = b'\x0d\x0a'  # CR+LF

        return header + encoded_id_bytes + extended_frame_flag + data_payload + tail

    def _send_and_receive(self, frame: bytes) -> Optional[bytes]:
        """コマンドを送信し、モーターからの応答を1フレーム受信します。"""
        if not self.ser or not self.ser.is_open:
            print("❌ エラー: シリアルポートが開かれていません。")
            return None

        self.ser.reset_input_buffer()  # 受信バッファをクリア
        self.ser.write(frame)

        # タイムアウト付きで応答を待機
        response = self.ser.read_until(b'\x0d\x0a', size=17)  # 応答は17バイトのはず

        if response and response.startswith(b'AT') and response.endswith(b'\r\n'):
            return response
        elif not response:
            print("❌ 検証失敗: モーターから時間内に応答がありませんでした。")
        else:
            print(f"❌ 検証失敗: 不正な形式の応答を受信しました -> {response.hex(' ')}")
        return None

    @staticmethod
    def _parse_and_validate_response(response_frame: bytes) -> bool:
        """
        応答フレームを解析し、モーターが正常な状態（RUNステータス、エラーなし）かを検証します。
        """
        if not response_frame or len(response_frame) < 17:
            print("❌ 検証失敗: 応答データが短すぎます。")
            return False

        # 応答IDからステータスとエラー情報を抽出 (取扱説明書 P.15参照)
        response_id_bytes = response_frame[2:6]
        can_id_29bit = int.from_bytes(response_id_bytes, 'big') >> 3

        status_val = (can_id_29bit >> 22) & 0b11
        error_flags = (can_id_29bit >> 16) & 0x3F

        try:
            status = MotorStatus(status_val)
        except ValueError:
            status = MotorStatus.UNKNOWN

        print(f"  [応答解析] ステータス: {status.name}, エラーコード: {error_flags}")

        # --- ここで厳密に検証 ---
        if status != MotorStatus.RUN:
            print(
                f"❌ 検証失敗: モーターがRUN状態ではありません (現在: {status.name})。"
            )
            return False

        if error_flags != 0:
            print(
                f"❌ 検証失敗: モーターがエラーを報告しています (コード: {error_flags})。"
            )
            return False

        return True

    def connect(self) -> bool:
        """シリアルポートに接続し、疎通確認を行います。"""
        print(f"--- ステップ1: 接続開始 (ポート: {self.port}) ---")
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1.0)
            print("✅ ポートを開きました。")
        except serial.SerialException as e:
            print(f"❌ ポートを開けませんでした: {e}")
            return False

        # 疎通確認コマンドを送信
        frame = self._create_frame(CommandType.GET_DEVICE_ID)
        response = self._send_and_receive(frame)

        if response:
            print("✅ 接続成功: モーターから正常な応答がありました。")
            return True
        else:
            print("❌ 接続失敗: モーターとの通信が確立できませんでした。")
            self.disconnect()
            return False

    def enable(self) -> bool:
        """モーターを有効化し、成功したか検証します。"""
        print("\n--- ステップ2: 有効化開始 ---")
        frame = self._create_frame(CommandType.ENABLE)
        response = self._send_and_receive(frame)

        if not response:
            return False  # 応答がなければ即失敗

        # 応答を解析して検証
        is_successful = self._parse_and_validate_response(response)

        if is_successful:
            print("✅ 有効化成功: モーターはエラーなくRUN状態に移行しました。")
        else:
            print("❌ 有効化失敗: モーターを正常に起動できませんでした。")

        return is_successful

    def disconnect(self) -> None:
        """シリアルポートを安全に切断します。"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("\nポートを閉じました。")


# --- このライブラリの使用例 ---
if __name__ == '__main__':
    SERIAL_PORT = "COM12"
    MOTOR_ID = 127

    motor = RobStrideMinimal(port=SERIAL_PORT, motor_id=MOTOR_ID)

    # ステップ1: 接続
    if motor.connect():
        # ステップ2: 有効化
        motor.enable()

    # 最後に必ず切断
    motor.disconnect()
