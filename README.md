# ATOM Matrix Y Tilt + RobStride 05 PD


ATOM Matrix の MPU6886 で Y 軸傾斜角をカルマンフィルタ推定し、角度と角速度の PD 制御で RobStride 05 を速度モード駆動する Arduino スケッチです。

詳細はコチラ
 https://homemadegarbage.com/codex01

## Hardware

- MCU: M5Stack ATOM Matrix
- Motor: RobStride 05, ID `0x7e`
- CAN transceiver: SN65HVD230 / VP230
- CAN speed: 1 Mbps
- CAN TX: GPIO32
- CAN RX: GPIO26

## Browser

1. スケッチを書き込む
2. PC またはスマートフォンを Wi-Fi `ATOM-RS05-ID`(任意)に接続する
3. パスワード `pass1234`(任意) を入力する
4. ブラウザで `http://192.168.4.1/` を開く

## Libraries

- `M5Atom`
- `KalmanFilter` by TKJ Electronics: https://github.com/TKJElectronics/KalmanFilter

## PD Control

- 速度指令: `speed = -(Kp * angleY + Kd * gyroY - Kw * wheelSpeed)`
- `gyroY`: カルマンフィルタのバイアス補正後レート
- `Kp` / `Kd` / `Kw`: ブラウザで `0.0` から `1.0` まで `0.05` 刻みで指定
- `wheelSpeed`: RobStride 05 の `0x701B` (`mechVel`) から読んだホイール回転速度
- 速度指令の上限: `+/-50 rad/s`
- フィードバック結果は符号反転して速度指令にします
- モータ Enable 条件: `|angleY| <= 1.0 deg` かつ IMU 正常
- モータ Disable 条件: `|angleY| >= 30.0 deg` または IMU 異常
- `1.0 deg` から `30.0 deg` の間では直前の Enable 状態を保持します
- CAN 指令更新周期: `500 us`, 2000 Hz
- 起動後 `1000 ms` 待ってから RobStride 05 の速度モード設定を送信します

回転方向が逆の場合は、`Kp` / `Kd` / `Kw` の符号を反転してください。

## Tilt Indicator

- 表示範囲: `-20 deg` から `+20 deg`
- 緑: `+/-1 deg` 以内
- 赤: `+/-15 deg` 以上
- 青: その他
- LED 輝度: `M5.dis.setBrightness(20)`

## API

- `GET /api/state`: 角度、角速度、ホイール速度、Kp/Kd/Kw、速度指令、CAN/IMU状態を返します
- `GET /api/cmd?kp=0.6`: `Kp` を設定します
- `GET /api/cmd?kd=1.0`: `Kd` を設定します
- `GET /api/cmd?kw=0.8`: `Kw` を設定します
