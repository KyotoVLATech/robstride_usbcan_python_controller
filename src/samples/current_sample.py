import time

from src.robstride import RobStride

# --- 設定項目 ---
SERIAL_PORT = "COM12"  # ご自身の環境に合わせてCOMポート名を指定してください
MOTOR_ID = 127  # 制御するモーターのCAN IDを指定してください


def main() -> None:
    """
    RobStrideモーターを電流モードで制御するメイン関数。
    安全のため、短時間の電流印加と停止を繰り返します。
    """
    print("--- RobStride 電流モード 制御サンプル (安全性考慮版) ---")
    print("警告: 電流モードの無負荷運転はモーターを暴走させる危険があります。")
    print("このサンプルは短時間の指令で安全に動作をデモします。")

    try:
        with RobStride(port=SERIAL_PORT, motor_id=MOTOR_ID) as motor:

            # --- ステップ1: Disable状態でモードを設定 ---
            if not motor.set_mode_current():
                print("エラー: 電流モードへの設定に失敗しました。処理を中断します。")
                return

            # --- ステップ2: モーターを有効化 ---
            if not motor.enable():
                print("エラー: モーターの有効化に失敗しました。処理を中断します。")
                return

            time.sleep(0.5)

            # --- ステップ3: 電流指令を送信 ---
            print("\n✅ 全ての準備が完了しました。モーターを動かします。")

            # 0.25Aの電流を2秒間印加 (ゆっくり正回転)
            motor.set_target_current(0.25)
            print("  -> 目標電流 0.25 A (2秒間)...")
            time.sleep(2)

            # 0Aで1秒間停止
            motor.set_target_current(0.0)
            print("  -> 停止 (1秒間)...")
            time.sleep(1)

            # -0.25Aの電流を2秒間印加 (ゆっくり逆回転)
            motor.set_target_current(-0.25)
            print("  -> 目標電流 -0.25 A (2秒間)...")
            time.sleep(2)

            # 最終的に0Aで停止
            motor.set_target_current(0.0)
            print("  -> 最終停止...")
            time.sleep(1)

            print("\n正常に処理が完了しました。")

    except Exception as e:
        print(f"\n予期せぬエラーが発生しました: {e}")


if __name__ == '__main__':
    main()
# python -m src.samples.current_sample
