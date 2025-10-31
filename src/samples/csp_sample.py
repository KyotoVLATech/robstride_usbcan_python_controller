import asyncio
import math

from src.robstride import RobStride, RobStrideController, RobStrideLimits

# --- 設定項目 ---
SERIAL_PORT = "COM5"  # ご自身の環境に合わせてCOMポート名を指定してください

# モーター制限設定
MOTOR_LIMITS = RobStrideLimits(
    csp_limit_spd=3.140,  # CSP速度制限 [rad/s]
    csp_limit_cur=0.5,  # CSP電流制限 [A]
)

# 3つのモーターの設定
MOTORS = [
    RobStride(id=1, offset=0.0, limits=MOTOR_LIMITS),
    # RobStride(id=5, offset=0.0, limits=MOTOR_LIMITS),
    # RobStride(id=6, offset=0.0, limits=MOTOR_LIMITS),
]


async def main() -> None:
    """
    3つのRobStrideモーターをCSPモードで同時制御するメイン関数。
    """
    print("--- RobStride CSPモード 3軸同時制御サンプル ---")

    try:
        async with RobStrideController(port=SERIAL_PORT, motors=MOTORS) as controller:

            # --- ステップ1: 全モーターをDisable状態でCSPモードに設定 ---
            print("\n🔧 全モーターをCSPモードに設定中...")
            for motor in MOTORS:
                if not await controller.set_mode_csp(motor.id):
                    print(f"エラー: モーター{motor.id}のCSPモード設定に失敗しました。")
                    return
                print(f"  ✅ モーター{motor.id}: CSPモード設定完了")

            # --- ステップ2: 全モーターを有効化 ---
            print("\n⚡ 全モーターを有効化中...")
            for motor in MOTORS:
                if not await controller.enable(motor.id):
                    print(f"エラー: モーター{motor.id}の有効化に失敗しました。")
                    return
                print(f"  ✅ モーター{motor.id}: 有効化完了")

            await asyncio.sleep(0.5)

            # --- ステップ3: CSP制限パラメータを適用 ---
            print("\n⚙️ CSP制限パラメータを設定中...")
            for motor in MOTORS:
                if not await controller.apply_csp_limits(motor.id):
                    print(f"エラー: モーター{motor.id}のCSP制限設定に失敗しました。")
                    return
                print(f"  ✅ モーター{motor.id}: CSP制限適用完了")

            # --- ステップ4: 同期された位置制御を実行 ---
            print("\n🎯 同期位置制御を開始します...")

            # パターン4: 原点復帰
            print("\n📍 パターン4: 全モーター原点復帰")
            for motor in MOTORS:
                await controller.set_target_position(motor.id, 0.0)
                print(f"  -> モーター{motor.id}: 目標位置 0.00 rad")
            await asyncio.sleep(3)

            # パターン1: 全モーターを45度に移動
            print("\n📍 パターン1: 全モーターを45度 (π/4 rad) に移動")
            target_positions = [-math.pi / 4, -math.pi / 4, -math.pi / 4]
            for i, motor in enumerate(MOTORS):
                await controller.set_target_position(motor.id, target_positions[i])
                print(
                    f"  -> モーター{motor.id}: 目標位置 {target_positions[i]:.2f} rad"
                )
            await asyncio.sleep(4)

            # パターン4: 原点復帰
            print("\n📍 パターン4: 全モーター原点復帰")
            for motor in MOTORS:
                await controller.set_target_position(motor.id, 0.0)
                print(f"  -> モーター{motor.id}: 目標位置 0.00 rad")
            await asyncio.sleep(3)

            # パターン5: 90Hz連続制御テスト（30秒間）
            print("\n🌊 パターン5: 90Hz連続正弦波制御テスト（30秒間）")
            frequency = 90  # Hz
            duration = 30  # 秒
            period = 1.0 / frequency  # 周期
            amplitude = math.pi * 3  # 振幅（±180度）
            sine_frequency = 0.1  # 正弦波の周波数（0.1Hz）

            start_time = asyncio.get_event_loop().time()
            last_send_time = start_time

            print(f"  -> 送信頻度: {frequency}Hz, 持続時間: {duration}秒")
            print(
                f"  -> 正弦波振幅: {math.degrees(amplitude):.1f}度, 周波数: {sine_frequency}Hz"
            )

            while (asyncio.get_event_loop().time() - start_time) < duration:
                current_time = asyncio.get_event_loop().time()

                # 90Hzの周期でコマンドを送信
                if (current_time - last_send_time) >= period:
                    elapsed = current_time - start_time

                    # 正弦波の目標位置を計算
                    target_position = amplitude * math.sin(
                        2 * math.pi * sine_frequency * elapsed
                    )

                    # 全モーターに同じ目標位置を設定
                    for motor in MOTORS:
                        await controller.set_target_position(motor.id, target_position)

                    # 進捗表示（0.5秒ごと）
                    if int(elapsed * 2) != int((elapsed - period) * 2):
                        print(
                            f"    時刻: {elapsed:.1f}s, 目標位置: {math.degrees(target_position):+6.1f}度"
                        )

                    last_send_time = current_time

                # CPUを少し休ませる
                await asyncio.sleep(0.001)

            # 最終的に原点に戻す
            print("  -> 原点復帰中...")
            for motor in MOTORS:
                await controller.set_target_position(motor.id, 0.0)
            await asyncio.sleep(2)

            print("\n✅ 全ての動作パターンが正常に完了しました。")

    except Exception as e:
        print(f"\n❌ 予期せぬエラーが発生しました: {e}")


if __name__ == '__main__':
    asyncio.run(main())
# 実行コマンド: python -m src.samples.csp_sample
