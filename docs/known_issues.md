# 既知の問題と修正方針

2026-08-19 の実機 bring-up (`docs/bringup_experiment_2026-08-19.md`) で判明したもの。
根拠となる実測値は全て実験記録側にある。優先度は **実機を動かすのにどれだけ効くか**で付けた。

---

## A. autonomy 側

### A-1. 【高】IMU のデータ化けを弾いていない

`/state/imu` に物理的にありえないサンプルが混入する (ロール ±170°、角速度 3軸とも ±35.6 rad/s
= BNO055 の int16 フルスケール、ノルム 0 のクォータニオン)。静止時でも約 30 秒に 1 回発生し、
運動中はさらに増える。

`navigator_node` / `auto_target_generator` / `rl_attitude_node` は `angular_velocity` を
ヨーレートとして、`orientation` を姿勢として直接使うため、**1 発のスパイクで制御が跳ねる**。

**修正**: IMU 購読側に共通のサニティフィルタを入れる。

- クォータニオンのノルムが 1 から大きく外れるサンプルを破棄 (`|‖q‖ − 1| > 0.01`)
- 角速度が物理的上限を超えるサンプルを破棄 (例 `|ω| > 10 rad/s`)
- 直前サンプルからの姿勢跳躍が閾値超えなら破棄 (例 1 制御周期で 30° 超)
- 破棄したら**直前の有効値を保持**し、破棄が連続したらフェイルセーフへ

置き場所は `umiusi_perception` ではなく autonomy 側の共通ユーティリティが妥当
(3 ノードが共有するため)。

### A-2. 【高】`max_rate_hz` のキャップがエイリアシングを起こす

`perception_node` の `max_rate_hz: 10.0` は「前回処理からの経過時間」で間引くだけなので、
入力が上限の 1〜2 倍のとき**かえってレートが落ちる**。実測で 15 Hz 入力 → 7.91 Hz、
13.4 Hz 入力 → 7.78 Hz。上限を外せば 19.8 Hz 出る。

**修正**: 次のいずれか。

- キャップを「目標周期に最も近いフレームを選ぶ」方式に変える (位相を追従させる)
- あるいは `max_rate_hz` の既定値を上げる (Pi 4 の実力は 19.8 Hz)。ただし
  フルスタック時は CPU が飽和するので **12–15 Hz あたりが実用点**

### A-3. 【解決済み】実機カメラ映像を perception に渡す経路が無い

`gst_camera_node` は GStreamer パイプラインを起動するだけで **ROS トピックに画像を出さない**。
一方 `perception_node` は `sensor_msgs/Image` を購読するため、両者が繋がっていなかった。

**`umiusi_autonomy/camera_bridge_node.py` を追加して解決** (2026-08-20)。
UI が既に見ている RTSP ストリームをそのまま tap するので、`sinsei_UMIUSI_control` は無改変、
カメラを二重に開くこともない。

```bash
ros2 run umiusi_autonomy camera_bridge_node --ros-args \
    -p rtsp_url:=rtsp://localhost:8554/cam1 -p width:=320 -p height:=240
```

**デコードと色変換/縮小はハードウェアに逃がすこと**が性能上の要点。実測:

| パイプライン | CPU |
|---|---:|
| `videoconvert ! videoscale` (software) | **102%** (CPU 律速でレートも 15→11.6 Hz に低下) |
| **`v4l2h264dec ! v4l2convert`** (hardware) | **33〜43%** |

既定は HW 経路で、開けない環境では software に自動フォールバックする。
publish サイズは `autonomy.yaml` の `frame_w/h` に合わせて 320x240 が既定
(640x480 の生 Image は 921 kB/frame あり、RELIABLE QoS では転送だけで頭打ちになる)。

### A-4. 【高】バンドル済み RL ポリシーが実機で読めない

`models/cruise_policy/final.zip` は **numpy 2.5.0** で保存されており、Pi (ROS Jazzy 標準の
**numpy 1.26.4**) では `ModuleNotFoundError: No module named 'numpy._core.numeric'` で失敗する。
`custom_objects` でも `numpy._core` のシムでも回避できない。

**修正**: 重み + 正規化統計を素形式へ書き出し、SB3/cloudpickle 非依存の torch 推論に切り替える。
検証済みの実装が `tools/policy_infer.py` にあり、**SB3 との出力差は 200 サンプルで 0.000e+00**。

- 書き出し: `umiusi_sim/export_policy.py` → `weights.pt` + `obs_norm.npz` + `meta.json`
- 実機側は **torch だけ**でよくなる (SB3・gymnasium・cloudpickle が不要になる)
- 副次効果として実機の依存が大幅に減る

`rl_attitude_node._ensure_model()` に、export ディレクトリがあればそちらを使うフォールバックを
足すのが最小差分。

### A-5. 【中】FSM が `umiusi_perception` に依存していることが分かりにくい

`navigator_node` と `auto_target_generator` も (perception だけでなく) `umiusi_perception` を
import する。未導入だと 10 秒ごとに
`cannot import the FSM from umiusi_perception` を吐き続けるだけで、ノードは起動したまま無音になる。

**幸い FSM 連鎖 (`autonomy → behavior → tracker → balloon_detector`) は numpy + scipy のみで
torch を必要としない**。実機に入れるのは `pip install --no-deps` で足りる。

**修正**: `package.xml` / README に明記し、起動時に 1 回だけ致命エラーとして扱う
(現状は WARN 相当の垂れ流しで見落としやすい)。

### A-6. 【低】`/cmd/target` が出ないのは正常だが分かりにくい

`core_autonomy.launch.py` で起動しても `/cmd/target` は出ない。core の BT が AUTO モードに
入って初めて `auto_target_generator` を activate する設計のため。**仕様どおり**だが、
動作確認時に「壊れている」と誤認しやすい。README に明記する。

---

## B. セットアップ / 実機環境側

### B-1. 【高】`usb_camera` のデバイス指定が誤っている

`params/cameras.yaml` の `usb_camera` が `/dev/video2` を指しているが、実機の `video2` は
**unicam (CSI) で H264 非対応**。`v4l2src ! video/x-h264` は開けない。

実機の対応表:

| デバイス | フォーマット |
|---|---|
| `/dev/video0` | MJPG, YUYV |
| `/dev/video2` | YUYV/UYVY/RGB (unicam) |
| **`/dev/video4`** | **H264 のみ** ← 正解 |

`docs/camera.md` も `/dev/video4` と明記している。**`/dev/video4` に戻す**。
正しいデバイスなら `PLAYING` に到達し、CPU は 12.3% しか使わない。

> デバイス番号は USB の挿し順で変わりうる。恒久的には udev ルールで
> `by-id` の固定名を作り、それを pipeline に書くのが安全。

### B-2. 【高】`gstreamer1.0-libcamera` が入っていない

前方 CSI カメラ (`pi_camera`) は `libcamerasrc` を使うが、実機に要素が無く
`GStreamer error: no element "libcamerasrc"` で FATAL 終了する。

```bash
sudo apt install gstreamer1.0-libcamera
```

### B-3. 【中】カメラ 1 本の失敗でノードが落ちる

`gst_camera_node` はパイプラインエラー時に `rclcpp::shutdown()` を呼ぶ。片方のカメラが
使えないと、そのノードが丸ごと消える。再試行かデグレード起動があると運用が楽になる
(`sinsei_UMIUSI_control` 側の変更)。

### B-4. 【高】有線直結では DHCP サーバと ufw の両方が必要

Pi は DHCP クライアントなので、PC 側で配らないと双方アドレスを取れない。さらに
**`ufw` が DDS を落とす**ため、開けないとトピックが一切見えない。

```bash
sudo nmcli con mod "Wired connection 1" ipv4.method shared
sudo ufw allow in on eno1 from 10.42.0.0/24
```

### B-5. 【高】`pip install torch` は aarch64 でも CUDA 版を引く

PyPI 既定の `torch` は aarch64 でも `+cu130` を取得し、`nvidia-*` 一式で **4.5 GB** を消費する
(GPU は無いので全て無駄)。さらに **`setuptools` が 84 に上がり colcon の制約 `<80` に違反する**。

```bash
# CPU 版を明示する
python3 -m pip install --user --break-system-packages \
    --index-url https://download.pytorch.org/whl/cpu torch
python3 -m pip install --user --break-system-packages "setuptools<80"
```

### B-6. 【中】実機に `pip` が入っていない

Ubuntu 24.04 の Pi イメージには `python3-pip` も `ensurepip` も無い。sudo を使わずに入れるなら:

```bash
curl -sSL -o /tmp/get-pip.py https://bootstrap.pypa.io/get-pip.py
python3 /tmp/get-pip.py --user --break-system-packages
export PATH="$HOME/.local/bin:$PATH"
```

### B-7. 【中】`ros2_control` がリアルタイム優先度を取れていない

起動のたびに出る:

```
Could not enable FIFO RT scheduling policy: with error number <1>(Operation not permitted)
```

100 Hz のハードウェアループが通常優先度で回っている。今回の測定では CPU 飽和下でも 50 Hz を
維持したが、**優先度による保護は効いていない**。

```bash
# /etc/security/limits.conf
pi  -  rtprio  99
pi  -  memlock unlimited
```

### B-8. 【高】CAN テレメトリが上流の未実装で成立しない

`sinsei_UMIUSI_control` の `can_model.cpp` が、実機の VESC(ATD) が送る status 種別
14/15/16 を扱わずエラーにする。逆に必要な 27 / Status6 は送られていない。
書き込み側も `ESC allowed` / `servo allowed` / `LED tape color` / `main power enabled` が TODO のまま。

**必要な作業** (`sinsei_UMIUSI_control` 側):

- `switch` に variant 1/2/3 (Status2/3/4) を追加する。Status4 には `temp_fet` があり有用
- あるいは **ATD 側の設定**で status 27 / Status6 を送出させる (どちらが正か要相談)
- 124 / 125 / 127 は ping には応答するが status を broadcast していない。ATD の
  「CAN status 送出」設定を 4 台で揃える必要がある

### B-9. 【中】`sinsei_umiusi_control` が古い

実機は PR#303 相当で、**ESC の推力符号修正 (PR#307) が未取り込み**。実機の推力方向に効くので
`main` へ更新すること。

---

## C. 対応の優先順

1. **B-8 / B-1 / B-2** — ハードが繋がらない・映らないので最優先
2. **A-4 / A-3** — これが無いと姿勢制御と perception が実機で成立しない
3. **A-1** — 安全性に直結 (スパイクで制御が跳ねる)
4. **B-4 / B-5 / B-6 / B-9** — セットアップの再現性
5. **A-2 / B-7** — 性能
6. **A-5 / A-6 / B-3** — 運用性・分かりやすさ
