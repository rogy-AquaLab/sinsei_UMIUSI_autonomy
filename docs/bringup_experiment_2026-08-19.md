# 実機 bring-up 実験記録 (2026-08-19)

core ユニット取り付け後の初回実機検証。対象機体 **`alexandrite`** (Raspberry Pi 4 Model B Rev 1.5)。
すべて実測値。推定値には「推定」と明記する。

## 0. 供試体

| 項目 | 値 |
|---|---|
| ホスト名 / MAC | `alexandrite` / `2c:cf:67:0c:50:1c` (Raspberry Pi Ltd) |
| OS / カーネル | Ubuntu 24.04.4 LTS / `6.8.0-1060-raspi` (aarch64) |
| SoC | Raspberry Pi 4 Model B Rev 1.5、4コア |
| RAM / ストレージ | 7.6 GiB / `/dev/sda2` 229 GB (空き 207 GB) |
| ROS 2 | Jazzy (`/opt/ros/jazzy`)、ワークスペース `~/ros2-ws` |

## 1. ネットワーク

有線直結。**Pi は DHCP クライアント**で、ケーブル両端が共にクライアントだったため通信不可だった
(静的 IP ではない)。PC 側を DHCP サーバ兼 NAT にして解決。

```bash
sudo nmcli con mod "Wired connection 1" ipv4.method shared   # PC=10.42.0.1/24
sudo ufw allow in on eno1 from 10.42.0.0/24                  # ← これが無いと DDS が通らない
```

- Pi は `10.42.0.135` / `alexandrite.local` を取得。RTT 0.19–0.24 ms、1000Mb/s full duplex。
- **`ufw` が DDS を遮断していた**。開ける前は PC↔Pi 双方向でトピックが一切見えなかった。
- リース固定が必要なら `/etc/NetworkManager/dnsmasq-shared.d/umiusi.conf` に
  `dhcp-host=2c:cf:67:0c:50:1c,10.42.0.135`。

## 2. CAN / VESC (ATD)

物理層は完全に正常。経路は `Pi → SPI → MCP2515 → can0 → VESC(ATD) ×4`。

| 項目 | 実測 |
|---|---|
| MCP2515 | `MCP2515 successfully initialized` |
| `can0` | UP / **ERROR-ACTIVE** / 500 kbps |
| エラーカウンタ | bus-errors 0 / arbit-lost 0 / error-warn 0 / error-pass 0 / bus-off 0 |
| ビットタイミング | tq 125 ns、16 tq/bit → 500 kbps (整合) |

`config.txt` は `oscillator=16000000` だが `ip -d link` は `clock 8000000` と表示する。これは
**正常**で、mcp251x ドライバは CAN クロックを発振子の 1/2 として報告する (TQ = 2·BRP/Fosc)。

### 4台すべて生きている

VESC PING (`0x11`) → PONG (`0x12`) を個別送信した結果:

| 宛先 CAN ID | 応答 | PONG ペイロード | 遅延 |
|---|---|---|---|
| 124 (0x7C) | ✅ | `7C 00` | ~0.2 ms |
| 125 (0x7D) | ✅ | `7D 00` | ~0.3 ms |
| 126 (0x7E) | ✅ | `7E 00` | ~0.2 ms |
| 127 (0x7F) | ✅ | `7F 00` | ~0.2 ms |

```bash
cansend can0 000011XX#00      # XX = 7C..7F。拡張IDは8桁hex必須
candump can0,00001200:1FFFFFFF
```

### ただし定期ステータスは 1台のみ

15 秒キャプチャ 2952 フレームは **全て CAN ID 126** で、種別は 9 / 14 / 15 / 16 のみ。
124・125・127 は ping には応答するが status を broadcast していない。

### ROS 制御層でテレメトリが成立しない (上流の未実装)

`can_model.cpp` の `switch` は variant `0`(Status=9) / `4`(Status5=27) / `5`(Status6) だけを扱い、
126 が実際に送る 14/15/16 は `default` → エラー。逆に必要な 27 と Status6 は送られていない。

60 秒間のエラー内訳:

| メッセージ | 件数 |
|---|---|
| `Unsupported VESC packet status variant received (VESC 3, variant index: 1)` | 16 |
| `Not implemented for LED tape color command` | 8 |
| `Not implemented for ESC allowed command` | 5 |
| `Not implemented for servo allowed command` | 4 |
| `Unsupported VESC packet status variant received (VESC 3, variant index: 3)` | 1 |

結果として `esc/rpm` · `esc/voltage` · `water_leaked` は取得できない。**CAN リンク自体は健全**なので
純粋にソフト側の対応漏れ。

## 3. IMU (BNO055)

`/state/imu` は `sensor_msgs/msg/Imu` を **50.0 Hz** で配信 (ImuState→Imu 移行が実機で裏取り済み)。

### 傾け→復帰試験

50 Hz 全数記録 2292 サンプル / 45 秒。基準 roll −0.49° / pitch +0.73°。

| 動作 | 最大傾斜 | 戻り値 | 復帰誤差 |
|---|---:|---:|---:|
| ピッチ前傾 | −46.68° | +1.76° | 約 +1.0° |
| ピッチ前傾 (2回目) | −30.71° | −0.28° | 約 −1.0° |
| ロール右 | +51.57° | −4.38° | 約 −3.9° |

±50° 振っても数度以内に復帰し、**ヒステリシス・ドリフトなし**。重力を絶対基準に使うロール/ピッチは
期待どおり。**ヨーは戻らない**(絶対基準が無いため正常)。

### データ化けが実在する

物理的にありえない単発サンプルが混入する。

```
20.0s roll= -2.07     20.5s roll=-169.67  ← 異常     21.0s roll= -3.69
35.8s roll= -5.32     36.3s roll=+125.76  ← 異常     36.8s roll= -5.50
```

- 前回試験では**角速度が3軸とも ±35.6 rad/s** に達した。これは BNO055 の int16 フルスケール
  (32767/16 = 2047.9 deg/s = 35.74 rad/s) とほぼ一致 = 読み出しの化け。
- 静止 60 秒 / 3000 サンプルでは**ジャイロ異常 0 件**、ただし
  **ノルム 0 のクォータニオンが 2 件**(約 30 秒に 1 回)。
- クォータニオンの符号反転 (q と −q) では説明できない (rpy 変換は ±q に対し不変)。

**影響**: `navigator_node` と `auto_target_generator` は `angular_velocity` をヨーレートに使うため、
35 rad/s のスパイクが1発入ると制御が跳ねる。受信側のサニティフィルタが必要。

## 4. Perception

モデルは `camp_mix.pt` (TinyBalloonNet, 446 KB)。実機カメラは ROS トピックを出さないため、
PC から sim 画像 (`ai/balloon/sim_eval/images` 200枚) を `/front_cam/image_raw` へ配信して測定。

| 入力レート | `max_rate_hz` | 実測認識周期 | 備考 |
|---:|---:|---:|---|
| 30 Hz | 0 (無制限) | **19.79 Hz** | **Pi 4 の上限**。CPU 262%/400%、48.7°C |
| 13.4 Hz | 0 | 13.38 Hz | 全フレーム追従 (演算は律速せず) |
| 13.4 Hz | 10 | 7.78 Hz | ← キャップのエイリアシング |
| 15 Hz | 10 | 7.91 Hz | 同上 |

**`max_rate_hz` は素朴な経過時間判定なので、入力が上限の 1〜2 倍のときエイリアシングで
かえってレートが落ちる**(15 Hz 入力 + 10 Hz キャップ → 7.9 Hz)。

転送側の制約: **640×480 では PC 送出が 6.5 Hz で頭打ち**(RELIABLE QoS + 921 KB/frame)。
**320×240 なら 30 Hz 出る**(`autonomy.yaml` の `frame_w/h` とも一致)。

検出例 (sim 画像): `colour: red, azimuth −0.072 rad, range 0.41 m, confidence 0.60`。

## 5. RL 姿勢制御ポリシー

`umiusi_rl_control/models/cruise_policy` (PPO, obs_mode=imu, task=attitude_velocity, 250万step)。

### 復元性・発散の検証 (MuJoCo 閉ループ)

機体を実際に傾けた状態から目標水平でロールアウト (400 step = 8 秒)。

| 軸 | 初期傾き | 最小誤差 | 最終誤差(末尾1秒) | 最大誤差 | 判定 |
|---|---:|---:|---:|---:|---|
| X (roll) | 10 / 20 / 30 / 45 / 60 / 90° | 0.7–1.6° | **2.1–2.7°** | 初期値を超えず | ✅ 全て復元 |
| Z (pitch) | 10 / 20 / 30 / 45 / 60 / 90° | 0.5–1.4° | **2.2–2.6°** | 初期値を超えず | ✅ 全て復元 |

**90° 倒しても水平復帰**。定常誤差 2–3° は成功判定閾値 (`ori_tol` 0.20 rad = 11.5°) の 1/5。
最大誤差が初期値を超えない = **オーバーシュートなし**。行動は `|a|max = 1.00` で飽和するが
振動・発散はしない。再現: `umiusi_sim/policy_restore_test.py`。

### 実機ではポリシーが読めない (numpy 非互換)

```
ModuleNotFoundError: No module named 'numpy._core.numeric'
```

ポリシー zip は **numpy 2.5.0** で保存され、Pi/ROS Jazzy は **numpy 1.26.4**。
`custom_objects` でも `numpy._core` シムでも回避不可 (次は `PCG64 is not a known BitGenerator`)。

**対処**: 重みと正規化統計を素形式へ書き出し、SB3/cloudpickle 非依存の torch 推論に置換。
`export_policy.py` で書き出し、`tools/policy_infer.py` で推論。
**SB3 との出力差は 200 サンプルで最大 0.000e+00 (完全一致)**。実機 numpy 1.26 で動作確認済み。

## 6. カメラ

`gst_camera_node` は **GStreamer パイプラインを ROS launch に載せるだけのアダプタ**で、
画像を ROS トピックに publish する経路が無い (パラメータは `pipeline` 文字列のみ)。
→ **perception に実機カメラ映像を渡す手段が現状存在しない**。

デバイスの実測:

| デバイス | 対応フォーマット | 対応 |
|---|---|---|
| `/dev/video0` | MJPG, YUYV | USB カメラ(下向き)の生出力 |
| `/dev/video2` | YUYV/UYVY/RGB (unicam=CSI) | **H264 非対応** |
| `/dev/video4` | **H264 のみ** | `v4l2src ! video/x-h264` が要求する形式 |

- `usb_camera` の `device=/dev/video2` は誤り。`docs/camera.md` と実機の両方が **`/dev/video4`** を指す。
  正しいデバイスで手動実行すると `PLAYING` に到達しエラー無し、**CPU 12.3%** のみ
  (カメラ内蔵 H264 ハードエンコードのため Pi は parse と RTSP 転送だけ)。
- `pi_camera` は **`no element "libcamerasrc"`** で FATAL。`gstreamer1.0-libcamera` 未導入。
- RTSP サーバ (`localhost:8554`) は稼働している。
- パイプラインエラー時に `rclcpp::shutdown()` を呼ぶため、**カメラ1本の失敗でノードが丸ごと落ちる**。

## 7. フルスタック同時稼働 (本命の測定)

`control`(カメラ2本) + `core_autonomy`(core BT + rosbridge/UI + perception + ATG) +
`rl_attitude_node`(姿勢制御ループ) を同時起動。**22 ノード**。

| トピック | カメラ・RL なし | **全系同時** | 変化 |
|---|---:|---:|---|
| `/state/imu` | 50.006 Hz | **50.093 Hz** | 維持 |
| `/state/thruster_state_all` | 50.189 Hz | **50.227 Hz** | 維持 |
| `/cmd/direct/…/output_lf` (姿勢制御) | 50.7 Hz | **33.777 Hz** | **−32%** |
| `/front_cam/image_raw` (PC 送出 15 Hz) | 15.023 Hz | **12.329 Hz** | −18% |
| `/perception_node/detections` | 7.910 Hz | **5.174 Hz** | −35% |

CPU / 熱:

| 条件 | アイドル | 温度 | スロットル |
|---|---:|---:|---|
| カメラ・RL なし | 34.0% | 46.7°C | なし |
| **全系同時** | **3.6%** | 50.1°C | `throttled=0x0` (なし) |

プロセス別 CPU (全系時): perception 78.5% / RL 58.3% / ros2_control 53.8% /
gst_camera 31.5%+14.8% / auto_target_generator 25.9% / rosbridge 21.9% / robot_strategy 6.2%

### 読み取れること

1. **C++ の `ros2_control` は CPU 飽和下でも 50 Hz を死守する**。
2. **Python 系が先に劣化する** — 姿勢制御 −32%、perception −35%、画像受信 −18%。
3. カメラのコストは USB(H264 パススルー) が 12–15%、CSI 相当(videoconvert+encode) が ~30%。
4. 熱には余裕がある (50°C、スロットルなし)。**制約は CPU であって温度ではない**。
5. `ros2_control` は **FIFO リアルタイム優先度を取得できていない**
   (`Could not enable FIFO RT scheduling policy: Operation not permitted`)。
   今回は 50 Hz を維持したが、優先度による保護は効いていない。

## 8. autonomy ドライ動確 (参考・カメラ無し時)

| 項目 | 結果 |
|---|---|
| `colcon build --packages-up-to umiusi_autonomy` | 5 パッケージ green / 64 秒 |
| lifecycle `configure → activate` | `active [3]` |
| `/cmd/target` | **50.02 Hz** で出力 (FSM 正常) |
| FSM 出力の妥当性 | 方位 +0.15 rad の赤風船に `orientation.z=0.5` + `velocity.z=−0.14` |
| navigator ドライ動作 | `publish:=false` でスラスタ出力なし |

`core_autonomy.launch.py` 経由では `/cmd/target` は出ない。**これは正常**で、
core の BT が AUTO モードに入って初めて `auto_target_generator` を activate する設計のため。

---

# 追補 (2026-08-20): 実カメラ経路と perception の高速化

## 9. 実カメラ → ROS の橋渡し

`camera_bridge_node` を追加し、**UI が見ている既存の RTSP をそのまま tap** して
`/front_cam/image_raw` に流す構成にした。`sinsei_UMIUSI_control` は無改変。

| パイプライン | CPU | 画像レート |
|---|---:|---:|
| `videoconvert ! videoscale` (software) | **102%** | 11.6 Hz (CPU 律速) |
| **`v4l2h264dec ! v4l2convert`** (hardware) | **33〜43%** | 12.2〜15.8 Hz |

デコードだけでなく**色変換と縮小もハードウェアに逃がすのが要点**。software の
`videoconvert`+`videoscale` は単体で 1 コアを食い潰す。

カメラ元設定の影響 (USB カメラは `/dev/video4` の H264 を使用):

| 元設定 | `gst_camera_node` CPU | ブリッジ CPU | 全体アイドル |
|---|---:|---:|---:|
| 1280x720@30 | 32.7% | 43.5% | 19.2% |
| 800x600@15 | **12.8%** | **33.3%** | **51.9%** |

`/dev/video4` が H264 で出せる解像度: 1920x1080 / 1280x720 / 800x600、いずれも 15/25/30 fps。

## 10. perception を 10 Hz に近づける

### 実測のまとめ

| 条件 | 認識周期 | 備考 |
|---|---:|---|
| perception 単独 (PC から 30 Hz 供給、無競合) | **19.8 Hz** | Pi 4 の素の上限 |
| control + カメラ + ブリッジ + perception | **8.31 Hz** | 実カメラ経路、`input_size=256`、アイドル 18.9%、perception は 185.7% CPU |
| フルスタック 22 ノード (姿勢制御 + UI + BT 込み) | **5.17 Hz** | アイドル 3.6% |

**`input_size` を下げると速くなるが精度を失う** (ラベル付き val での評価):

| `input_size` | F1 | precision | recall | 認識周期 |
|---:|---:|---:|---:|---:|
| **256** (既定) | **0.69** | 0.66 | 0.72 | 5.6–8.3 Hz |
| 192 | 0.55 (−20%) | 0.58 | 0.54 | **10.4 Hz** |
| 160 | 0.42 (−39%) | 0.46 | 0.39 | — |

192 は 10 Hz に届くが **recall が 0.72 → 0.54** に落ちる (風船を 4 分の 1 余計に見逃す)。
速度のために精度を捨てる取引であり、ただ乗りではない。

`sanitise_near` の on/off は認識周期に有意差なし (6.04 / 5.42 Hz、測定ノイズの範囲)。

### 10 Hz に必要なもの (優先順)

素の上限が 19.8 Hz なので、**`input_size=256` のままでも 10 Hz は原理的に届く**。
実測 8.31 Hz との差は約 2 割で、そのときアイドルが 18.9% 残っていた。

1. **全部を同時に回さない** — フルスタックの 5.17 Hz には姿勢制御 (58%)、rosbridge/UI (22%)、
   BT + ATG (32%) が乗っている。UI を使わないときは `use_rosbridge:=false`、
   `rl_attitude_node` の `control_hz` を下げるだけで 0.5〜0.8 コア空く
2. **ブリッジは必ずハードウェア経路で** — software だと単体で 1 コア消える
3. **カメラ元設定を落とす** — 800x600@15 で `gst_camera_node` が 32.7% → 12.8%
4. **ブリッジの `max_rate_hz` を perception が消化できる値に合わせる** —
   捨てるフレームの publish と DDS 転送に CPU を使わない
5. ここまでで足りなければ `input_size=192` (F1 0.69 → 0.55 を受け入れる場合のみ)
6. 恒久策としては、より小さいモデルの再学習や int8 量子化

> 注意: 測定中に `ros2 launch` の `timeout` が切れて上流が止まると、入力レートより
> 認識周期が高く出るなど辻褄の合わない値になる。周期を測るときは上流の生存を必ず確認すること。
