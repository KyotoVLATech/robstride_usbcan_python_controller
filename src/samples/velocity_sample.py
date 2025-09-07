import time

from src.robstride import RobStride

# --- 設定項目 ---
SERIAL_PORT = "COM12"  # ご自身の環境に合わせてCOMポート名を指定してください
MOTOR_ID = 127  # 制御するモーターのCAN IDを指定してください


def main() -> None:
    """
    RobStrideモーターを速度モードで制御するメイン関数。
    """
    print("--- RobStride 速度モード 制御サンプル ---")

    try:
        with RobStride(port=SERIAL_PORT, motor_id=MOTOR_ID) as motor:

            # --- ステップ1: Disable状態でモードを設定 ---
            if not motor.set_mode_velocity():
                print("エラー: 速度モードへの設定に失敗しました。処理を中断します。")
                return

            # --- ステップ2: モーターを有効化 ---
            if not motor.enable():
                print("エラー: モーターの有効化に失敗しました。処理を中断します。")
                return

            time.sleep(0.5)

            # --- ステップ3: Enable状態でパラメータを設定 ---
            # これらは仕様書通りの手順です
            if not motor.set_velocity_limit_cur(5.0):  # 電流制限 5.0A
                print("エラー: 電流制限の設定に失敗しました。処理を中断します。")
                return

            if not motor.set_velocity_acceleration(10.0):  # 加速度 10.0 rad/s^2
                print("エラー: 加速度の設定に失敗しました。処理を中断します。")
                return

            # --- ステップ4: 速度指令を送信 ---
            print("\n✅ 全ての準備が完了しました。モーターを動かします。")

            # 正方向 (3.0 rad/s) へ回転
            motor.set_target_velocity(3.0)
            print("  -> 目標速度 3.00 rad/s へ...")
            time.sleep(3)

            # 逆方向 (-3.0 rad/s) へ回転
            motor.set_target_velocity(-3.0)
            print("  -> 目標速度 -3.00 rad/s へ...")
            time.sleep(3)

            # 停止
            motor.set_target_velocity(0.0)
            print("  -> 停止します...")
            time.sleep(1)

            print("\n正常に処理が完了しました。")

    except Exception as e:
        print(f"\n予期せぬエラーが発生しました: {e}")


if __name__ == '__main__':
    main()
# python -m src.samples.velocity_sample
