# RP2040 Taiko Controller

RP2040-Zeroを使用した Nintendo Switch / PC 対応の太鼓コントローラー変換基板です。

## 主な機能

- SNES（SFC）純正コントローラー対応
- ピエゾセンサー4ch入力対応
- Nintendo Switchモード対応（ポッ拳コントローラー互換）
- PCキーボードモード対応
- OLED表示対応（任意）
- EEPROM設定保存対応
- Webモニター対応
- Web経由の感度調整対応
- https://puniyounyo.github.io/rp2040taiko/taiko_control.html
- RAWデータ出力対応
- PC／Switch感度プロファイル分離（V1.0〜）

---

## GPIO割り当て

### SNESコントローラー

| 信号 | GPIO |
|--------|--------|
| LATCH | GP0 |
| CLOCK | GP1 |
| DATA | GP2 |
| VCC | 3.3V |
| GND | GND |

### OLED（SSD1306）

| OLED | GPIO |
|--------|--------|
| SDA | GP4 |
| SCL | GP5 |
| VCC | 3.3V |
| GND | GND |

### ピエゾセンサー

| センサー | GPIO | 役割 |
|----------|--------|--------|
| CH0 | GP26(ADC0) | 左カ |
| CH1 | GP27(ADC1) | 左ドン |
| CH2 | GP28(ADC2) | 右ドン |
| CH3 | GP29(ADC3) | 右カ |

---

## 操作方法

### 通常起動

電源投入時にSTARTボタンを押していない場合

- Switchモードで起動
- NeoPixel：青

### PCモード

電源投入時にSTARTボタンを押しながら起動

- キーボードエミュレーションモード
- NeoPixel：緑

---

## 設定モード

PCモード: SELECTボタン短押し / Switchモード: SELECTボタン1秒以上長押し

- OLED上でパラメータ設定
- EEPROM保存対応（PC用／Switch用プロファイルは別々に保存）
- NeoPixel：赤

---

## GOD CALIB（神自動キャリブレーション）

V0.9で廃止されました（V1.0にもありません）。

---

## Webモニターコマンド

### 設定取得

```text
GETS
```

### 設定変更

```text
SENS:0=80:1=160:2=160:3=80
```

### EEPROM保存

```text
SAVE
```

### PCプロファイルをSwitchプロファイルへコピー（V1.0〜）

```text
COPY2SW
```

### RAWデータ出力開始

```text
RAWON
```

### RAWデータ出力停止

```text
RAWOFF
```

---

## パラメータ一覧

| No | 名称 | 内容 |
|----|------|------|
| 0 | L_EDGE | 左カ感度 |
| 1 | L_FACE | 左ドン感度 |
| 2 | R_FACE | 右ドン感度 |
| 3 | R_EDGE | 右カ感度 |
| 4 | FACE_INTV | 面入力ガード |
| 5 | EDGE_INTV | 縁入力ガード |
| 6 | CROSS_IG | 面⇔縁クロストーク抑制 |
| 7 | HOLD_MS | 押下保持時間 |
| 8 | FACE_SGL | 面自己抑制 |
| 9 | EDGE_SGL | 縁自己抑制 |
| 10 | FACE_LR | 左右面ガード |
| 11 | EDGE_LR | 左右縁ガード |
| 12 | CROSS_RATIO | 比率ゲート（最強ピーク比、V1.0〜） |
| 13 | JUDGE_WIN | 判定収集ウィンドウ ms（V1.0〜、0で無効） |

※ V1.0 から PC用／Switch用で感度プロファイルを分けて保存します。Web（PCモード）で調整した値は、コマンド `COPY2SW` でSwitch用プロファイルへコピーできます。Switchモードでは SELECT 長押し（1秒）で設定モードに入れます。

---

## リポジトリ構成

- `rp2040taiko_V1.0.ino` : 現行版ファームウェア
- `taiko_control.html` : Web設定＆モニター（GitHub Pagesで公開）
- `3DModels/` : スペーサー類のSTL／F3D
- `archive/` : 過去バージョン（V0.1〜V0.9）

---

## 変更履歴

| Version | 主な変更内容 |
|----------|-------------|
| V0.1 | 初版リリース。SNES入力、ピエゾ4ch、Switch/PCモード、OLED表示、EEPROM保存機能を実装。 |
| V0.2 | 判定パラメータを見直し、誤検知低減とレスポンス向上を実施。 |
| V0.3 | Webモニター対応。シリアル経由でヒット情報を取得可能に。 |
| V0.4 | Web設定変更機能・RAWデータ出力機能・PairDelayGate判定を実装。 |
| V0.5 | GOD CALIB（神自動キャリブレーション）を実装。感度・クロストーク対策値を自動算出しEEPROMへ自動保存。 |
| V0.6 | キャリブレーション方式と表示ロジックを整理し、二段階EMA（Cascade EMA）フィルタを導入。 |
| V0.7 | 「Ultimate」版として再構成。Switch／PCキーボード／Debugの3モード化、Smart Noise Gate＋EMA＋Pair Delayを検出構成として明示。 |
| V0.8 | 反転マッピング版。物理ピン→論理チャンネルの反転（physical 0→logical 3）を明示し、パラメータ名を論理順（D/F/J/K）に統一。※ファイル内ヘッダーの版表記はv0.7のまま |
| V0.9 | Switchモードでシリアルを完全停止して認識を安定化。HITイベント送信、RAW送信の40msピーク保持化、設定値の0〜255制限、設定モード中のベースライン更新。自動キャリブレーション廃止、設定モードはPCモードのみに。2色OLEDの色境界に合わせた表示配置に調整。 |
| V1.0 | 検出を正方向デルタ方式へ回帰し、比率ゲート（CROSS_RATIO／JUDGE_WIN）を追加。PC／Switch感度プロファイル分離、Switchモードでの設定モード対応、Web設定ページの14項目対応。 |

---

## バージョン概要

- V0.1 : 基本機能完成版
- V0.2 : 安定化版
- V0.3 : Webモニター対応版
- V0.4 : Webキャリブレーション対応版
- V0.5 : 神自動キャリブレーション対応版
- V0.6 : Cascade EMA・キャリブレーション整理版
- V0.7 : Ultimate再構成版（3モード）
- V0.8 : 反転マッピング版
- V0.9 : Switch認識安定化版（シリアル完全停止）
- V1.0 : 比率ゲート・プロファイル分離版（現行）
