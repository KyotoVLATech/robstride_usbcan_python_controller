import asyncio
import math

from src.robstride import RobStride, RobStrideController, RobStrideLimits

# --- 設定項目 ---
SERIAL_PORT = "COM5"  # ご自身の環境に合わせてCOMポート名を指定してください

# モーター制限設定
MOTOR_LIMITS = RobStrideLimits(
    pp_vel_max=10.0,  # PP最大速度 [rad/s]
    pp_acc_set=10.0,  # PP加速度 [rad/s^2]
    pp_limit_cur=5.0,  # PP電流制限 [A]
)

# モーターの設定
MOTORS = [
    RobStride(id=1, offset=0.0, limits=MOTOR_LIMITS),
    # RobStride(id=5, offset=0.0, limits=MOTOR_LIMITS),
    # RobStride(id=6, offset=0.0, limits=MOTOR_LIMITS),
]


async def main() -> None:
    """
    RobStrideモーターをPPモード（Position Profile）で制御するメイン関数。
    """
    print("--- RobStride PPモード（Position Profile）制御サンプル ---")

    try:
        async with RobStrideController(port=SERIAL_PORT, motors=MOTORS) as controller:

            # --- ステップ1: 全モーターをDisable状態でPPモードに設定 ---
            print("\n🔧 全モーターをPPモードに設定中...")
            for motor in MOTORS:
                if not await controller.set_mode_pp(motor.id):
                    print(f"エラー: モーター{motor.id}のPPモード設定に失敗しました。")
                    return
                print(f"  ✅ モーター{motor.id}: PPモード設定完了")

            # --- ステップ2: 全モーターを有効化 ---
            print("\n⚡ 全モーターを有効化中...")
            for motor in MOTORS:
                if not await controller.enable(motor.id):
                    print(f"エラー: モーター{motor.id}の有効化に失敗しました。")
                    return
                print(f"  ✅ モーター{motor.id}: 有効化完了")

            await asyncio.sleep(0.5)

            # --- ステップ3: PP制限パラメータを適用 ---
            print("\n⚙️ PP制限パラメータを設定中...")
            for motor in MOTORS:
                if not await controller.apply_pp_limits(motor.id):
                    print(f"エラー: モーター{motor.id}のPP制限設定に失敗しました。")
                    return
                print(f"  ✅ モーター{motor.id}: PP制限適用完了")

            # --- ステップ4: 位置制御パターンを実行 ---
            print("\n🎯 位置制御パターンを開始します...")

            # パターン1: 原点復帰
            print("\n📍 パターン1: 原点復帰")
            for motor in MOTORS:
                await controller.set_target_position(motor.id, 0.0)
                print(f"  -> モーター{motor.id}: 目標位置 0.00 rad")
            await asyncio.sleep(3)

            # パターン2: 90度回転
            print("\n📍 パターン2: 90度 (π/2 rad) 回転")
            target_pos = math.pi / 2
            for motor in MOTORS:
                await controller.set_target_position(motor.id, target_pos)
                print(f"  -> モーター{motor.id}: 目標位置 {target_pos:.2f} rad")
            await asyncio.sleep(4)

            # パターン3: 180度回転
            print("\n📍 パターン3: 180度 (π rad) 回転")
            target_pos = math.pi
            for motor in MOTORS:
                await controller.set_target_position(motor.id, target_pos)
                print(f"  -> モーター{motor.id}: 目標位置 {target_pos:.2f} rad")
            await asyncio.sleep(4)

            # パターン4: 原点復帰
            print("\n📍 パターン4: 原点復帰")
            for motor in MOTORS:
                await controller.set_target_position(motor.id, 0.0)
                print(f"  -> モーター{motor.id}: 目標位置 0.00 rad")
            await asyncio.sleep(3)

            print("\n✅ 全ての動作パターンが正常に完了しました。")

    except Exception as e:
        print(f"\n❌ 予期せぬエラーが発生しました: {e}")


if __name__ == '__main__':
    asyncio.run(main())
# 実行コマンド: python -m src.samples.pp_sample
