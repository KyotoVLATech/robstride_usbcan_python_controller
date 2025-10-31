import time

from src.robstride import RobStride, RobStrideController, RobStrideLimits

# --- 設定項目 ---
SERIAL_PORT = "COM5"  # ご自身の環境に合わせてCOMポート名を指定してください

# モーター制限設定
MOTOR_LIMITS = RobStrideLimits(
    velocity_limit_cur=2.0,  # Velocity電流制限 [A]
    velocity_acc_rad=5.0,  # Velocity加速度 [rad/s^2]
)

# モーターの設定
MOTORS = [
    RobStride(id=1, offset=0.0, limits=MOTOR_LIMITS),
    # RobStride(id=5, offset=0.0, limits=MOTOR_LIMITS),
    # RobStride(id=6, offset=0.0, limits=MOTOR_LIMITS),
]


def main() -> None:
    """
    RobStrideモーターをVelocity制御モードで制御するメイン関数。
    """
    print("--- RobStride Velocity制御モードサンプル ---")

    try:
        with RobStrideController(port=SERIAL_PORT, motors=MOTORS) as controller:

            # --- ステップ1: 全モーターをDisable状態でVelocity制御モードに設定 ---
            print("\n🔧 全モーターをVelocity制御モードに設定中...")
            for motor in MOTORS:
                if not controller.set_mode_velocity(motor.id):
                    print(
                        f"エラー: モーター{motor.id}のVelocity制御モード設定に失敗しました。"
                    )
                    return
                print(f"  ✅ モーター{motor.id}: Velocity制御モード設定完了")

            # --- ステップ2: 全モーターを有効化 ---
            print("\n⚡ 全モーターを有効化中...")
            for motor in MOTORS:
                if not controller.enable(motor.id):
                    print(f"エラー: モーター{motor.id}の有効化に失敗しました。")
                    return
                print(f"  ✅ モーター{motor.id}: 有効化完了")

            time.sleep(0.5)

            # --- ステップ3: Velocity制限パラメータを適用 ---
            print("\n⚙️ Velocity制限パラメータを設定中...")
            for motor in MOTORS:
                if not controller.apply_velocity_limits(motor.id):
                    print(
                        f"エラー: モーター{motor.id}のVelocity制限設定に失敗しました。"
                    )
                    return
                print(f"  ✅ モーター{motor.id}: Velocity制限適用完了")

            # --- ステップ4: 速度制御パターンを実行 ---
            print("\n🎯 速度制御パターンを開始します...")

            # パターン1: 正方向回転
            print("\n📍 パターン1: 正方向回転 (0.5 rad/s)")
            target_velocity = 15.7
            for motor in MOTORS:
                controller.set_target_velocity(motor.id, target_velocity)
                print(f"  -> モーター{motor.id}: 目標速度 {target_velocity:.1f} rad/s")
            time.sleep(3)

            # パターン2: 停止
            print("\n📍 パターン2: 停止 (0.0 rad/s)")
            target_velocity = 0.0
            for motor in MOTORS:
                controller.set_target_velocity(motor.id, target_velocity)
                print(f"  -> モーター{motor.id}: 目標速度 {target_velocity:.1f} rad/s")
            time.sleep(2)

            # パターン3: 負方向回転
            print("\n📍 パターン3: 負方向回転 (-0.8 rad/s)")
            target_velocity = -0.8
            for motor in MOTORS:
                controller.set_target_velocity(motor.id, target_velocity)
                print(f"  -> モーター{motor.id}: 目標速度 {target_velocity:.1f} rad/s")
            time.sleep(3)

            # パターン4: 最終停止
            print("\n📍 パターン6: 最終停止")
            for motor in MOTORS:
                controller.set_target_velocity(motor.id, 0.0)
                print(f"  -> モーター{motor.id}: 目標速度 0.0 rad/s")
            time.sleep(2)

            print("\n✅ 全ての動作パターンが正常に完了しました。")

    except Exception as e:
        print(f"\n❌ 予期せぬエラーが発生しました: {e}")


if __name__ == '__main__':
    main()
# 実行コマンド: python -m src.samples.velocity_sample
