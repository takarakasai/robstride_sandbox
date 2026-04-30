# robstride_sandbox

Robstride Edulite05 モーターコントローラー (Rust + SocketCAN)

## 構成

```
src/
  lib.rs          - ライブラリ エントリポイント
  protocol.rs     - CAN プロトコル (フレーム構築・解析)
  motor.rs        - モーター制御 API
  error.rs        - エラー型
  main.rs         - CLI ツール
examples/
  status.rs             - ステータス読み取り
  position_control.rs   - 位置制御
  velocity_control.rs   - 速度制御
  torque_control.rs     - トルク制御
setup_can.sh            - SLCAN → SocketCAN セットアップスクリプト
stop_can.sh             - CAN インターフェース停止スクリプト
```

## 前提条件

- Linux (SocketCAN 対応)
- USB-CAN アダプタ (SLCAN 互換, `/dev/ttyUSB0`)
- Rust toolchain (1.70+)
- `can-utils` パッケージ (`sudo apt install can-utils`)

## CAN インターフェースのセットアップ

### SLCAN アダプタの場合 (推奨)

```bash
# セットアップスクリプトを使用 (1Mbps, Robstride デフォルト)
sudo ./setup_can.sh

# ビットレートを指定する場合
sudo ./setup_can.sh 500000

# 停止
sudo ./stop_can.sh
```

### ネイティブ CAN アダプタの場合

```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

# 確認
ip -details link show can0
```

### CAN FD を使う場合

CAN FD で通信するには、インターフェースを FD 対応で立ち上げる必要があります
（アービトレーション用 `bitrate` に加えてデータ位相用 `dbitrate` を指定し `fd on`）。
送信フレームは BRS（ビットレート切替）有効で送られるため、`dbitrate` が必須です。

```bash
sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can0 up
```

その上で、TUI では `f` キー、CLI では `--fd` フラグで CAN FD 通信に切り替えます
（下記参照）。

## ビルド

```bash
cargo build --release
```

## CLI 使用方法

基本構文:

```
robstride_sandbox [OPTIONS] <COMMAND>
```

### グローバルオプション

| オプション | 短縮 | デフォルト | 説明 |
|-----------|------|-----------|------|
| `--interface` | `-i` | `can0` | CAN インターフェース名 |
| `--motor-id` | `-m` | `1` | 対象モーターの CAN ID (1-127) |
| `--host-id` | — | `0xFD` | ホスト側 CAN ID |
| `--fd` | — | `false` | 全通信を CAN FD（BRS 有効）で行う |

### コマンド一覧

#### バス診断 (モーターID 指定不要)

```bash
# バス上の全モーターをスキャン (ID 1〜127)
cargo run -- -i can0 scan

# スキャン範囲を指定
cargo run -- -i can0 scan --from 1 --to 32

# ID あたりのタイムアウトを変更 (デフォルト: 50ms)
cargo run -- -i can0 scan --timeout 200

# CAN バスのトラフィックをパッシブ監視 (送信なし)
cargo run -- -i can0 dump

# 監視時間を指定 (デフォルト: 5秒)
cargo run -- -i can0 dump --duration 10
```

#### モーター基本操作

```bash
# ステータス読み取り
cargo run -- -i can0 -m 1 status

# モーター有効化
cargo run -- -i can0 -m 1 enable

# モーター無効化
cargo run -- -i can0 -m 1 disable

# 現在位置をゼロ点に設定
cargo run -- -i can0 -m 1 set-zero
```

#### 位置制御

```bash
# 3.14 rad へ移動 (速度制限: デフォルト 5 rad/s)
cargo run -- -i can0 -m 1 move-to 3.14

# 速度制限を指定
cargo run -- -i can0 -m 1 move-to 3.14 --speed 2.0
```

到達するまでリアルタイムでフィードバックを表示し、目標位置に達すると自動で停止します。

#### 速度制御

```bash
# 2 rad/s で回転 (Ctrl+C で停止)
cargo run -- -i can0 -m 1 spin 2.0

# 5秒間だけ回転
cargo run -- -i can0 -m 1 spin 2.0 --duration 5.0

# 逆回転
cargo run -- -i can0 -m 1 spin -3.0
```

#### トルク制御

```bash
# 0.5 Nm を印加 (Ctrl+C で停止)
cargo run -- -i can0 -m 1 torque 0.5

# 3秒間だけ印加
cargo run -- -i can0 -m 1 torque 0.5 --duration 3.0
```

#### MIT モード制御

位置・速度・トルク・PD ゲインを同時に指定する高度な制御モードです。

```bash
# 位置 1.0 rad, Kp=50, Kd=1.0
cargo run -- -i can0 -m 1 mit --pos 1.0 --kp 50.0 --kd 1.0

# トルクフィードフォワード付き
cargo run -- -i can0 -m 1 mit --pos 0.0 --vel 0.0 --kp 30.0 --kd 0.5 --torque 0.2

# 全パラメータゼロ (ステータス読み取りのみ)
cargo run -- -i can0 -m 1 mit
```

#### パラメータ読み取り

```bash
cargo run -- -i can0 -m 1 read-param mech_pos    # 機械角位置
cargo run -- -i can0 -m 1 read-param mech_vel    # 機械角速度
cargo run -- -i can0 -m 1 read-param iq_filt     # フィルタ後電流
cargo run -- -i can0 -m 1 read-param vbus        # バス電圧
```

エイリアスも使えます: `position`, `velocity`, `current`, `voltage`

#### 連続モニタリング

```bash
# 100ms 間隔でモニタリング (デフォルト)
cargo run -- -i can0 -m 1 monitor

# 50ms 間隔
cargo run -- -i can0 -m 1 monitor --interval 50
```

## サンプル実行

```bash
cargo run --example status
cargo run --example position_control
cargo run --example velocity_control
cargo run --example torque_control
```

## デバッグログ

```bash
RUST_LOG=debug cargo run -- -i can0 -m 1 status
RUST_LOG=debug cargo run -- -i can0 scan
```

## トラブルシューティング

### モーターが見つからない場合

1. **CAN インターフェースの確認**
   ```bash
   ip link show can0
   ```

2. **パッシブ監視でバス上のトラフィックを確認**
   ```bash
   cargo run -- -i can0 dump
   # または
   candump can0
   ```

3. **ビットレートの確認** — Robstride のデフォルトは 1Mbps

4. **配線の確認** — CAN-H, CAN-L, GND が正しく接続されているか

5. **電源の確認** — モーターに電源が供給されているか

6. **終端抵抗** — バスの両端に 120Ω の終端抵抗があるか

### SLCAN デバイスの確認

```bash
ls -la /dev/ttyUSB*
dmesg | tail -20    # USB デバイス認識ログ
```

## プロトコル概要

Robstride CAN 拡張フレーム (29bit ID):

| Bits 28-24 | Bits 23-16 | Bits 15-8  | Bits 7-0   |
|------------|------------|------------|------------|
| 通信タイプ (5bit) | データフィールド (8bit) | ターゲットID (8bit) | ソースID (8bit) |

### 通信タイプ一覧

| コード | 名称 | 説明 |
|--------|------|------|
| 0x00 | GetDeviceInfo | デバイス情報取得 |
| 0x01 | MotorFeedback | モーターフィードバック (応答) |
| 0x02 | MotorControl | MIT モード制御 |
| 0x03 | MotorEnable | モーター有効化 |
| 0x04 | MotorDisable | モーター無効化 |
| 0x05 | SetMechanicalZero | 機械的ゼロ点設定 |
| 0x06 | ChangeCanId | CAN ID 変更 |
| 0x07 | ReadParam | パラメータ読み取り |
| 0x08 | WriteParam | パラメータ書き込み |

### 制御モード

| モード | 値 | 説明 |
|--------|---|------|
| MIT | 0 | 位置・速度・トルク・PD ゲイン同時指定 |
| Position | 1 | 位置制御モード |
| Velocity | 2 | 速度制御モード |
| Torque | 3 | トルク（電流）制御モード |

### Edulite05 パラメータ範囲

| パラメータ | 最小値 | 最大値 | 単位 |
|-----------|--------|--------|------|
| Position | -12.566 | 12.566 | rad |
| Velocity | -30.0 | 30.0 | rad/s |
| Torque | -12.0 | 12.0 | Nm |
| Kp | 0.0 | 500.0 | — |
| Kd | 0.0 | 5.0 | — |
