import time

from src.robstride import RobStride, RobStrideController

# --- 設定項目 ---
SERIAL_PORT = "COM5"  # ご自身の環境に合わせてCOMポート名を指定してください

# モーターの設定
MOTORS = [
    RobStride(id=1, offset=0.0),
    # RobStride(id=5, offset=0.0),
    # RobStride(id=6, offset=0.0),
]


def main() -> None:
    """
    RobStrideモーターをCurrent制御モードで制御するメイン関数。
    """
    print("--- RobStride Current制御モードサンプル ---")

    try:
        with RobStrideController(port=SERIAL_PORT, motors=MOTORS) as controller:

            # --- ステップ1: 全モーターをDisable状態でCurrent制御モードに設定 ---
            print("\n🔧 全モーターをCurrent制御モードに設定中...")
            for motor in MOTORS:
                if not controller.set_mode_current(motor.id):
                    print(
                        f"エラー: モーター{motor.id}のCurrent制御モード設定に失敗しました。"
                    )
                    return
                print(f"  ✅ モーター{motor.id}: Current制御モード設定完了")

            # --- ステップ2: 全モーターを有効化 ---
            print("\n⚡ 全モーターを有効化中...")
            for motor in MOTORS:
                if not controller.enable(motor.id):
                    print(f"エラー: モーター{motor.id}の有効化に失敗しました。")
                    return
                print(f"  ✅ モーター{motor.id}: 有効化完了")

            time.sleep(0.5)

            # --- ステップ3: Current制御モードでは特別な制限設定は不要 ---
            print("\n⚙️ Current制御モードでは特別な制限設定は不要です")
            print("   Current制御では、set_target_current()で直接電流値を指定します")

            # --- ステップ4: 電流制御パターンを実行 ---
            print("\n🎯 電流制御パターンを開始します...")

            # パターン1: 正方向トルク
            print("\n📍 パターン1: 正方向トルク (0.3A)")
            target_current = 0.3
            for motor in MOTORS:
                controller.set_target_current(motor.id, target_current)
                print(f"  -> モーター{motor.id}: 目標電流 {target_current:.1f} A")
            time.sleep(2)

            # パターン2: 電流停止
            print("\n📍 パターン2: 電流停止 (0.0A)")
            target_current = 0.0
            for motor in MOTORS:
                controller.set_target_current(motor.id, target_current)
                print(f"  -> モーター{motor.id}: 目標電流 {target_current:.1f} A")
            time.sleep(1)

            # パターン3: 負方向トルク
            print("\n📍 パターン3: 負方向トルク (-0.3A)")
            target_current = -0.3
            for motor in MOTORS:
                controller.set_target_current(motor.id, target_current)
                print(f"  -> モーター{motor.id}: 目標電流 {target_current:.1f} A")
            time.sleep(2)

            # パターン4: 段階的電流変化
            print("\n📍 パターン4: 段階的電流変化")
            current_steps = [0.1, 0.2, 0.4, 0.2, 0.0, -0.2, -0.4, -0.2, 0.0]
            for step_current in current_steps:
                print(f"  -> 目標電流: {step_current:.1f} A")
                for motor in MOTORS:
                    controller.set_target_current(motor.id, step_current)
                time.sleep(1)

            # パターン5: 最終停止
            print("\n📍 パターン5: 最終停止")
            for motor in MOTORS:
                controller.set_target_current(motor.id, 0.0)
                print(f"  -> モーター{motor.id}: 目標電流 0.0 A")
            time.sleep(1)

            print("\n✅ 全ての動作パターンが正常に完了しました。")

    except Exception as e:
        print(f"\n❌ 予期せぬエラーが発生しました: {e}")


if __name__ == '__main__':
    main()
# 実行コマンド: python -m src.samples.current_sample
