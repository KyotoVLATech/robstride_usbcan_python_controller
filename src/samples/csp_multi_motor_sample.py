import math
import time

from src.robstride import RobStride, RobStrideController

# --- 設定項目 ---
SERIAL_PORT = "COM6"  # ご自身の環境に合わせてCOMポート名を指定してください

# 3つのモーターの設定
MOTORS = [
    RobStride(id=4, max_pos=2 * math.pi, min_pos=0.0, max_speed=1.0, offset=0.0),
    RobStride(id=5, max_pos=2 * math.pi, min_pos=0.0, max_speed=1.0, offset=0.0),
    RobStride(id=6, max_pos=2 * math.pi, min_pos=0.0, max_speed=1.0, offset=0.0),
]


def main() -> None:
    """
    3つのRobStrideモーターをCSPモードで同時制御するメイン関数。
    """
    print("--- RobStride CSPモード 3軸同時制御サンプル ---")

    try:
        with RobStrideController(port=SERIAL_PORT, motors=MOTORS) as controller:

            # --- ステップ1: 全モーターをDisable状態でCSPモードに設定 ---
            print("\n🔧 全モーターをCSPモードに設定中...")
            for motor in MOTORS:
                if not controller.set_mode_csp(motor.id):
                    print(f"エラー: モーター{motor.id}のCSPモード設定に失敗しました。")
                    return
                print(f"  ✅ モーター{motor.id}: CSPモード設定完了")

            # --- ステップ2: 全モーターを有効化 ---
            print("\n⚡ 全モーターを有効化中...")
            for motor in MOTORS:
                if not controller.enable(motor.id):
                    print(f"エラー: モーター{motor.id}の有効化に失敗しました。")
                    return
                print(f"  ✅ モーター{motor.id}: 有効化完了")

            time.sleep(0.5)

            # --- ステップ3: 各モーターの速度制限を設定 ---
            print("\n⚙️ 速度制限を設定中...")
            velocity_limits = [1.0, 1.0, 1.0]  # 各モーターの速度制限 [rad/s]
            for i, motor in enumerate(MOTORS):
                if not controller.set_csp_velocity_limit(motor.id, velocity_limits[i]):
                    print(f"エラー: モーター{motor.id}の速度制限設定に失敗しました。")
                    return
                print(
                    f"  ✅ モーター{motor.id}: 速度制限 {velocity_limits[i]:.1f} rad/s"
                )

            # --- ステップ4: 同期された位置制御を実行 ---
            print("\n🎯 同期位置制御を開始します...")

            # パターン4: 原点復帰
            print("\n📍 パターン4: 全モーター原点復帰")
            for motor in MOTORS:
                controller.set_target_position(motor.id, 0.0)
                print(f"  -> モーター{motor.id}: 目標位置 0.00 rad")
            time.sleep(3)

            # パターン1: 全モーターを45度に移動
            # print("\n📍 パターン1: 全モーターを45度 (π/4 rad) に移動")
            # target_positions = [-math.pi / 4, -math.pi / 4, -math.pi / 4]
            # for i, motor in enumerate(MOTORS):
            #     controller.set_target_position(motor.id, target_positions[i])
            #     print(
            #         f"  -> モーター{motor.id}: 目標位置 {target_positions[i]:.2f} rad"
            #     )
            # time.sleep(4)

            # パターン4: 原点復帰
            print("\n📍 パターン4: 全モーター原点復帰")
            for motor in MOTORS:
                controller.set_target_position(motor.id, 0.0)
                print(f"  -> モーター{motor.id}: 目標位置 0.00 rad")
            time.sleep(3)

            print("\n✅ 全ての動作パターンが正常に完了しました。")

    except Exception as e:
        print(f"\n❌ 予期せぬエラーが発生しました: {e}")


if __name__ == '__main__':
    main()
# 実行コマンド: python -m src.samples.csp_multi_motor_sample
