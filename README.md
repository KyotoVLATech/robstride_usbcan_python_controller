# RobStride USBCAN Python Controller (Async Version)

RobStrideモーターの非同期制御用Pythonライブラリです。複数のシリアルバス間の並行処理と、単一バス内の安全な逐次処理を実現します。

## 🚀 新機能: 非同期処理対応

- **異なるシリアルバス間の並行処理**: 複数のUSBCANアダプタを使用時に真の並行処理でスループット向上
- **単一バス内の安全な混線防止**: `async with self.lock` によるロック機能で通信の安全性を確保
- **効率的なI/O待機**: `await`によるCPUリソースの効率的利用
- **柔軟な制御パターン**: `asyncio.gather()`による複雑な制御シーケンスの実現

## 📋 前提条件

- **OS**: Windows 11
- **ハードウェア**: RobStride 00
- **ファームウェア**: rs00_0.0.3.6.bin
- **通信モジュール**: RobStride公式USBCANモジュール
- **パッケージマネージャ**: uv
- **Python**: 3.11以上（asyncio対応）

## 📦 依存関係

- `pyserial-asyncio`: 非同期シリアル通信
- `asyncio`: 非同期処理フレームワーク

## 🛠️ セットアップ

1. **リポジトリのクローン**
   ```bash
   git clone https://github.com/KyotoVLATech/robstride_usbcan_python_controller.git
   cd robstride_usbcan_python_controller
   ```

2. **依存関係のインストール**
   ```bash
   uv sync
   ```

3. **ハードウェア接続**
   - RobStride公式USBCANモジュールをRobStrideモーターに接続
   - モーターに適切な電源を供給
   - USBCANモジュールをPCのUSBポートに接続

## 🎮 基本的な使用方法

### 基本的な非同期制御

```python
import asyncio
from src.robstride import RobStride, RobStrideController, RobStrideLimits

async def main():
    # モーター設定
    motors = [RobStride(id=1, offset=0.0)]
    
    # 非同期with文でコントローラーを使用
    async with RobStrideController(port="COM5", motors=motors) as controller:
        # モーターの有効化
        await controller.enable(1)
        
        # CSPモードに設定
        await controller.set_mode_csp(1)
        
        # 目標位置の設定（90度回転）
        await controller.set_target_position(1, 1.57)
        
        # 2秒待機
        await asyncio.sleep(2.0)

if __name__ == '__main__':
    asyncio.run(main())
```

### 制限パラメータ付きの制御

```python
import asyncio
from src.robstride import RobStride, RobStrideController, RobStrideLimits

async def main():
    # モーター制限の設定
    limits = RobStrideLimits(
        csp_limit_spd=3.14,  # CSP速度制限 [rad/s]
        csp_limit_cur=0.5,   # CSP電流制限 [A]
    )
    
    motors = [RobStride(id=1, offset=0.0, limits=limits)]
    
    async with RobStrideController(port="COM5", motors=motors) as controller:
        await controller.set_mode_csp(1)
        await controller.enable(1)
        
        # 制限パラメータを適用
        await controller.apply_csp_limits(1)
        
        # 制御を実行
        await controller.set_target_position(1, 1.57)

if __name__ == '__main__':
    asyncio.run(main())
```

## 🔄 対応制御モード

### 1. CSP (Cyclic Synchronous Position) モード
```python
await controller.set_mode_csp(motor_id)
await controller.apply_csp_limits(motor_id)  # オプション
await controller.set_target_position(motor_id, position_rad)
```

### 2. PP (Profile Position) モード
```python
await controller.set_mode_pp(motor_id)
await controller.apply_pp_limits(motor_id)  # オプション
await controller.set_target_position(motor_id, position_rad)
```

### 3. Velocity モード
```python
await controller.set_mode_velocity(motor_id)
await controller.apply_velocity_limits(motor_id)  # オプション
await controller.set_target_velocity(motor_id, velocity_rad_per_s)
```

### 4. Current モード
```python
await controller.set_mode_current(motor_id)
await controller.set_target_current(motor_id, current_ampere)
```

## 🏃‍♂️ サンプルコードの実行

### 基本サンプル
```bash
# CSPモード制御サンプル
uv run -m src.samples.csp_sample

# 電流制御サンプル
uv run -m src.samples.current_sample

# PPモード制御サンプル
uv run -m src.samples.pp_sample

# 速度制御サンプル
uv run -m src.samples.velocity_sample
```

### 接続テスト
```bash
# 基本的な通信テスト
uv run connection_test.py
```

## 🔧 高度な使用方法

### 複数バスの並行制御

```python
import asyncio
from src.robstride import RobStride, RobStrideController

async def control_bus_a(controller):
    """バスAの制御タスク"""
    await controller.set_mode_csp(1)
    await controller.enable(1)
    for i in range(100):
        position = math.sin(i * 0.1) * math.pi / 4
        await controller.set_target_position(1, position)
        await asyncio.sleep(0.01)

async def control_bus_b(controller):
    """バスBの制御タスク"""
    await controller.set_mode_csp(2)
    await controller.enable(2)
    for i in range(100):
        position = math.cos(i * 0.1) * math.pi / 4
        await controller.set_target_position(2, position)
        await asyncio.sleep(0.01)

async def main():
    # 2つの異なるバスを並行制御
    controller_a = RobStrideController(port="COM5", motors=[RobStride(id=1)])
    controller_b = RobStrideController(port="COM6", motors=[RobStride(id=2)])
    
    async with controller_a, controller_b:
        # 真の並行処理で両バスを同時制御
        await asyncio.gather(
            control_bus_a(controller_a),
            control_bus_b(controller_b)
        )

if __name__ == '__main__':
    asyncio.run(main())
```

## 📊 制限パラメータ

### CSPモード制限
- `csp_limit_spd`: 速度制限 [rad/s]
- `csp_limit_cur`: 電流制限 [A]

### PPモード制限
- `pp_vel_max`: 最大速度 [rad/s]
- `pp_acc_set`: 加速度 [rad/s²]
- `pp_limit_cur`: 電流制限 [A]

### Velocityモード制限
- `velocity_limit_cur`: 電流制限 [A]
- `velocity_acc_rad`: 加速度 [rad/s²]

## ⚠️ 重要な注意事項

1. **非同期with文の使用**: `with` ではなく `async with` を使用してください
2. **await の使用**: 全ての制御メソッドには `await` が必要です
3. **混線防止**: 同一バス内では自動的にロックが適用され、逐次実行されます
4. **エラーハンドリング**: 通信エラーに対する適切な例外処理を実装してください

## 🔍 トラブルシューティング

### 接続エラー
```python
# 接続テストで基本的な通信を確認
uv run connection_test.py
```

### COMポートの確認
- デバイスマネージャーでUSBCANモジュールのCOMポート番号を確認
- サンプルコード内の `SERIAL_PORT` を適切な値に設定

### ファームウェアバージョンの確認
- 対応ファームウェア: rs00_0.0.3.6.bin
- モーターのファームウェアが最新であることを確認

## 📚 API リファレンス

### RobStrideController主要メソッド

- `async connect() -> bool`: 接続開始
- `async disconnect() -> None`: 接続終了
- `async enable(motor_id: int) -> bool`: モーター有効化
- `async disable(motor_id: int) -> None`: モーター無効化
- `async set_mode_csp(motor_id: int) -> bool`: CSPモード設定
- `async set_mode_pp(motor_id: int) -> bool`: PPモード設定
- `async set_mode_velocity(motor_id: int) -> bool`: Velocityモード設定
- `async set_mode_current(motor_id: int) -> bool`: Currentモード設定
- `async set_target_position(motor_id: int, position_rad: float) -> None`: 目標位置設定
- `async set_target_velocity(motor_id: int, velocity: float) -> None`: 目標速度設定
- `async set_target_current(motor_id: int, current: float) -> None`: 目標電流設定