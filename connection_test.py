import time

import serial

# --- 設定項目 ---
# ご自身の環境に合わせて変更してください
SERIAL_PORT = "COM12"  # COMポート名（linuxなら/dev/ttyUSB0など）
BAUDRATE = 921600  # ボーレート
MOTOR_CAN_ID = 127  # 確認したいモーターのCAN ID（初期値は127）
HOST_CAN_ID = 253  # ホスト側(PC)のID (任意だが0以外)
# -----------------


def create_at_command(
    command_type: int, motor_id: int, host_id: int, data_payload: int
) -> bytes:
    """ATモードのコマンドフレームを生成する"""

    # 1. CAN拡張IDを構築 (29ビット)
    # Bit 28-24: 通信タイプ, Bit 23-8: ホストID, Bit 7-0: モーターID
    can_id_29bit = (command_type << 24) | (host_id << 8) | motor_id

    # 2. USB-CANツールの特殊なエンコーディングを適用
    # マニュアルの変換例から、29ビットIDの末尾にバイナリの'100'を追加すると推測
    encoded_id_32bit = (can_id_29bit << 3) | 0b100

    # 3. ATコマンドの各パーツをバイト列で作成
    header = b'\x41\x54'  # "AT"
    encoded_id_bytes = encoded_id_32bit.to_bytes(4, 'big')
    extended_frame_flag = b'\x08'
    data = data_payload.to_bytes(8, 'big')  # データフィールド
    tail = b'\x0d\x0a'  # CR+LF

    return header + encoded_id_bytes + extended_frame_flag + data + tail


# --- メイン処理 ---
if __name__ == "__main__":
    # 疎通確認用の「デバイスID取得（タイプ0）」コマンドを生成
    # このコマンドはモーターにデータを要求するだけで、状態を変更しないため安全です
    # データペイロードは0でなければなりません
    command_to_send = create_at_command(
        command_type=0, motor_id=MOTOR_CAN_ID, host_id=HOST_CAN_ID, data_payload=0
    )

    print(f"シリアルポート {SERIAL_PORT} を開きます...")

    try:
        with serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1.0) as ser:
            print("ポートを開きました。")

            print(f"送信コマンド (HEX): {command_to_send.hex(' ')}")
            ser.write(command_to_send)
            print("コマンドを送信しました。応答を待っています...")

            time.sleep(0.5)  # 応答を待つための短いウェイト

            response = ser.read(100)  # 十分な長さのデータを読み取る

            if response:
                print("\n✅ 応答を受信しました！ 疎通成功です。")
                print(f"受信データ (HEX): {response.hex(' ')}")
            else:
                print("\n❌ 応答がありませんでした。")

    except serial.SerialException as e:
        print("\nエラー: シリアルポートを開けませんでした。")
        print(f"詳細: {e}")
        print("COMポート名が正しいか、デバイスがPCに接続されているか確認してください。")
