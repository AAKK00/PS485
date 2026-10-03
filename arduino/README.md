# elephant_robot_position_sensor — RS-485 関節ポテンショメータ

ATtiny202 + MAX485E の自作基板を Dynamixel Protocol 2.0 スレーブとして動かし、
ポテンショメータの角度を RS-485 バス経由で読むシステム。
(最終更新: 2026-07-17、動作確認済み)

```
[PC/USB] — [Arduino Uno (master)] — [MAX485] ==A/B== [基板 ID100] == [基板 ID101]
                                                      (ATtiny202+MAX485E, 10V給電)
```

- プロトコル: Dynamixel Protocol 2.0、57600 baud 固定
- ID 100 / ID 101、モデル番号 0x4B41
- コントロールテーブル: addr 65 = LED (1byte RW, 現基板は物理LED無し) / addr 132 = Present Position (2byte RO, ADC生値 0–1023)

## ⚠ 一番大事な注意 (今回のハマりどころ)

**この基板は USART0 をデフォルトピン (PA6=TX, PA7=RX) で配線している。**
DynamixelSlave ライブラリは元々 PORTMUX で ALT ピン (PA1/PA2) に切り替えていたため、
スレーブが完全沈黙する不具合があった。現在はライブラリに `begin(bool alt_pins)` を追加済みで、
**この基板用スケッチは必ず `dxl.begin(false)` で呼ぶこと。**

### 基板の実ピン配置 (ATtiny202 SOIC-8)

| 物理pin | ポート | 接続先 |
|---|---|---|
| 1 | VCC | 5.3V (TPS562200 降圧出力) |
| 2 | PA6 | tx1 → MAX485 DI (USART0 デフォルト TXD) |
| 3 | PA7 | rx1 ← MAX485 RO (USART0 デフォルト RXD) |
| 4 | PA1 | 未接続 (次リビジョンで LED 予定) |
| 5 | PA2 | V_out ← ポテンショ wiper (R5+C5 の RC 経由, AIN2) |
| 6 | PA0 | UPDI |
| 7 | PA3 | ena → MAX485 DE+RE (HIGH=送信) |
| 8 | GND | GND |

## スケッチ / ライブラリ

| 場所 | 役割 |
|---|---|
| `joint_pot_slave/` | 基板用スレーブファーム。**焼く前に `MY_ID` を 100 か 101 に設定** |
| `joint_pot_master/` | Uno 用マスター。SoftwareSerial で RS-485、USB はデバッグ専用 (9600) |
| `dxl_bus_diag/` | Uno 用バススニッファ (トラブル時の切り分け用) |
| `dxl_slave_test/` | Uno 用の最小 PING / READ / WRITE 通信テスト。配線・モニタ速度はスケッチ冒頭を参照 |
| `attiny_beacon_test/` | ATtiny 用自己診断ファーム (TX/RX 経路の単体検証) |
| `libraries/DynamixelSlave/` | スレーブ側プロトコル実装。`begin(false)` でデフォルトピン |

### 開発環境の準備

- Arduino IDE に **megaTinyCore** と **Arduino AVR Boards** をインストールする。
- 同梱の `libraries/DynamixelSlave/` を Arduino のスケッチブック配下の `libraries/` にコピーする。PS485 基板には、`begin(false)` に対応したこの版を使う。
- `joint_pot_master` には **Dynamixel2Arduino 0.8.1** が必要 (元の開発環境で使用した版)。Arduino IDE のライブラリマネージャからインストールする。SoftwareSerial は Arduino AVR Boards に含まれる。
- 各フォルダ内の同名 `.ino` を開く。`joint_pot_slave` と `attiny_beacon_test` は ATtiny202、それ以外は Arduino Uno 向け。

`libraries/DynamixelSlave/examples/JointPotentiometer/` は旧配線 (UART が PA1/PA2、ポテンショメータが PA6) の例。現 PS485 基板には `joint_pot_slave/` を使用する。

## マスター (Uno) の配線

| Uno | MAX485 モジュール |
|---|---|
| pin 10 | RO |
| pin 11 | DI |
| pin 8 | DE (RE は GND に落とす。DE と結んでも可) |
| 5V/GND | VCC/GND |

- モニタは **9600 baud**。バスは 57600 (SoftwareSerial の実用上限なので上げない)
- 診断用 `dxl_bus_diag` を使う時だけ RO → pin 0 に差し替え。**pin 0 に RO が刺さっていると Uno への書き込みが失敗する**ので upload 前に抜くこと

## スレーブの焼き方

1. Arduino IDE: megaTinyCore / **ATtiny202** / **Clock: 20 MHz internal**
2. クロック設定を変えたら **Burn Bootloader** (ヒューズ反映) を忘れずに
3. `joint_pot_slave.ino` の `MY_ID` を基板ごとに設定 (100 or 101)
4. UPDI (jtag2updi) で書き込み。ターゲット給電は 10V 入力か UPDI 側 5V の**どちらか片方だけ**
5. **Uno を jtag2updi プログラマとして使うと Uno のスケッチが消える**。ATtiny を焼いたら Uno に master を焼き直すこと

## 性能

- 1 READ トランザクション ≈ 4.7ms (27 bytes @57600) → 1軸 ~200Hz / 2軸 ~100Hz が理論値
- 高速化するなら: バスを 115200 に (slave の `BAUD_REG = 4×20MHz/baud` を再計算、master は hardware UART か U2D2 が必要)

## 既知の事項

- `ping FAIL err=0x42` = UNKNOWN_MODEL_NUMBER。モデル 0x4B41 が Dynamixel2Arduino の既知リストに無いだけで**通信は成功している**(master スケッチでは対処済み)
- **ブロードキャスト ping に応答しない**仕様 → Dynamixel Wizard のスキャンには映らないことがある。ID 直接指定なら見える
- バイトスタッフィング省略 → 現在の control table では問題ないが、値を追加する時は FF FF FD が並ばないか要確認
- ID 100 の個体でポテンショ読み値が ±25 ほど揺れる (ID 101 は ±3)。ポットの接触/配線を疑う。スレーブ側 EMA 実装は未着手
- ADC のソースインピーダンスはデータシート推奨 10kΩ 以下。ピン直近の C5 (100n) がチャージリザーバとして効くので R5 は 10k まで許容、4.7k が無難

## 次リビジョン TODO

- [ ] **PA1 に LED** (1k〜2.2k 直列、アクティブ High)。起動点滅 + addr 65 連動でデバッグが激変する
- [ ] R5 を 10Ω → **4.7k** (fc ≈ 340Hz のまともなローパスになる)
- [ ] TPS562200 のフィードバックを調整して **5.0V ぴったり**に (実測 5.3V は MAX485E 推奨上限 5.25V を微超過)
