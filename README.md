# UGOKU One Inverted Pendulum (対向2輪・倒立振子 サンプル)
UGOKU One（ESP32）で対向2輪ロボットをDCモーター車輪で自立させる、最小限の倒立振子プログラムです。UGOKU PadからPIDゲインをリアルタイムに調整できます。

### UGOKU Pad
https://ugoku-lab.github.io/ugokupad.html

Console内の「ESP32 Arduino Sample」を使用

<img src="https://github.com/user-attachments/assets/a0c7ed43-5082-4802-9647-cbb8cc861142" width="200">
<img src="https://github.com/user-attachments/assets/578605c3-9ea8-434b-b564-59bf12aa8233" width="200">

### できること
- BMI270(IMU)で姿勢（ピッチ）を推定（コンプリメンタリフィルタ）
- PID制御で左右DCモーターを同方向駆動し、倒立を保持
- UGOKU Padから Kp/Ki/Kd と角度トリムを調整

### ピン配置
| 機能 | ピン |
| ------------- | ------------- |
| デジタル出力  | 27 |
| 測距モジュール | 26 |
| RCサーボ | 14 |
| ローテーションサーボ | 12 |
| I2C SDA | 21 |
| I2C SCL | 22 |

### 使用ライブラリ
- Arduino標準ライブラリ
- Arduino_BMI270_BMM150（Arduino公式 IMU ライブラリ）

インストール方法: Arduino IDE の「ライブラリを管理…」で「Arduino_BMI270_BMM150」を検索してインストール。

### 動作確認
ESP32-WROOM-32E、ESP32-WROVER-Eで動作確認済み

## 使い方
1. ハードウェア接続
	- BMI270/BMM150（I2C）: VCC->3.3V, GND->GND, SDA->GPIO21, SCL->GPIO22
	- モータードライバ配線（ソース参照）
	  - MD1: IN1=GPIO32, IN2=GPIO33
	  - MD2: IN1=GPIO5,  IN2=GPIO13
	- ロボットは+Xが前方、+Zが上になるように搭載（スケッチ内で軸の符号は調整可能）

2. スケッチ書き込み
	- `UGOKU-One_Inverted-pendulum.ino` をボードに書き込み
	- シリアルモニタ 115200bps を開くとIMU初期化状態などが確認できます

3. UGOKU Pad と接続・チューニング
	- UGOKU Padの Console「ESP32 Arduino Sample」で接続
	- チャンネル割り当て（0..180 がスライダ/スティックの範囲）
	  - ch2: Kp（0..40）
	  - ch3: Ki（0..2）
	  - ch4: Kd（0..2）
	  - ch5: 角度トリム（-90..+90 度; 90が0度）
	- 目安: まず Ki=0 で Kp を上げ、次に Kd を入れて揺れを抑え、最後に Ki を少しずつ加える

4. テレメトリ
	- ch20 にピッチ角（0..180; 90が0度）を送信

## 軸・向きの調整
- モーターの回転向きが逆の場合は `UGOKU-One_Inverted-pendulum.ino` 内の `#define MD1_DIR` / `#define MD2_DIR` を `1` または `-1` に変更してください。
- ピッチ角の符号が合わない場合は `atan2f(-ax, sqrt(ay^2+az^2))` の `-ax` の符号を反転して調整します。

## メモ
- 余計な機能（サーボ/LED/PSD等）は削除し、倒立制御に必要な最低限の処理のみ実装しています。
- モータPWMは `MotorDriver.*` を利用。停止方式やデューティ最小値は同ファイル内の定義を調整してください。


