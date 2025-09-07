import math
import time

from robstride import RobStride

# --- 設定項目 ---
SERIAL_PORT = "COM12"  # ご自身の環境に合わせてCOMポート名を指定してください
MOTOR_ID = 127  # 制御するモーターのCAN IDを指定してください


def main() -> None:
    """
    RobStrideモーターをPPモードで制御するメイン関数。
    with構文を使用し、安全な接続・切断を保証します。
    """
    print("--- RobStride PPモード 制御サンプル ---")

    try:
        # with構文により、ブロックを抜ける際に自動でdisable()とdisconnect()が呼ばれます
        with RobStride(port=SERIAL_PORT, motor_id=MOTOR_ID) as motor:

            # --- ステップ1: Disable状態でモードを設定 ---
            if not motor.set_mode_pp():
                print("エラー: PPモードへの設定に失敗しました。処理を中断します。")
                return  # 関数を抜ける (withブロックが終了処理をハンドル)

            # --- ステップ2: モーターを有効化 ---
            if not motor.enable():
                print("エラー: モーターの有効化に失敗しました。処理を中断します。")
                return

            time.sleep(0.5)  # 有効化後の安定待ち

            # --- ステップ3: Enable状態でパラメータを設定 ---
            if not motor.set_pp_velocity(5.0):
                print("エラー: 最大速度の設定に失敗しました。処理を中断します。")
                return

            if not motor.set_pp_acceleration(10.0):
                print("エラー: 加速度の設定に失敗しました。処理を中断します。")
                return

            # --- ステップ4: 位置指令を送信 ---
            print("\n✅ 全ての準備が完了しました。モーターを動かします。")

            # 90度 (π/2 rad) の位置へ移動
            motor.set_target_position(math.pi / 2)
            print("  -> 目標位置 {math.pi/2:.2f} rad へ移動中...")
            time.sleep(3)  # 移動が完了するまで待機

            # 0度の位置へ戻る
            motor.set_target_position(0.0)
            print("  -> 目標位置 0.00 rad へ移動中...")
            time.sleep(3)

            print("\n正常に処理が完了しました。")

    except Exception as e:
        print(f"\n予期せぬエラーが発生しました: {e}")


if __name__ == '__main__':
    main()
