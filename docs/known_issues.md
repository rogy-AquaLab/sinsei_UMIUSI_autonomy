# 既知の問題と修正方針

2026-08-19 の実機 bring-up (`docs/bringup_experiment_2026-08-19.md`) で判明したもの。
根拠となる実測値は全て実験記録側にある。優先度は **実機を動かすのにどれだけ効くか**で付けた。

---

## A. autonomy 側

### A-1. 【解決済み】IMU のデータ化けを弾いていない

`/state/imu` に物理的にありえないサンプルが混入する。実機 (BNO055) で確認したもの:

* **ノルムが 0 のクォータニオン** — 静止 60 秒で 2 件 (約 30 秒に 1 回)
* **角速度が 3 軸とも ±35.6 rad/s** — int16 フルスケール (32767/16 = 2047.9 deg/s) と一致
* **0.5 秒で −3° → −170° → −4° の姿勢跳躍** — 運動中に出やすい

`navigator_node` / `auto_target_generator` / `rl_attitude_node` はいずれも角速度を
ヨーレートとして、姿勢をそのまま制御・観測に使うため、**1 発のスパイクで制御が跳ねる**。

**実装済み**: `umiusi_rl_control/imu_sanity.py` の `ImuSanity`。3 ノードすべての IMU
コールバックに入れた。判定は「疑わしきは捨てる」で、捨てたら直前の有効値を保持し、
連続して捨て続けると `stale` が立つ。

| パラメータ | 既定 | 意味 |
|---|---:|---|
| `imu_max_gyro` | 10.0 rad/s | これを超える角速度は破棄 (フルスケールは 35.74) |
| `imu_max_step_deg` | 30.0 deg | 1 サンプルの姿勢跳躍上限。50 Hz なら 1500 deg/s 相当 |

ROS 非依存の純関数なので単体テストできる。**実機で観測した実際の化け値を使った
テスト 10 件が通っている** (`umiusi_rl_control/test/test_imu_sanity.py`)。
符号反転 (q と −q) を急変と誤判定しないこと、正常な運動 (1 サンプル 30° まで) を
通すことも確認済み。

#### 実機で検証した結果 (2026-08-21、機体を手で 150 秒振った bag)

**閾値は妥当だった。棄却しているのは「速い運動」ではなく明確な読み出し化けだった。**

- 実際に動かしたときの角速度は最大 **4.6 rad/s** (閾値 10 に対し 2.2 倍の余裕)、
  1 サンプルの姿勢変化は最大 **6.1 deg** (閾値 30 に対し 4.9 倍の余裕)。
- **BNO055 の姿勢と角速度はよく整合している**。棄却されなかった 7453 サンプルで、
  実際の姿勢変化と「角速度 × dt」の差は **中央値 0.004° / p99 1.25°**。ノイジーではない。
- 棄却されたのは 150 秒で 33 件 (0.44%):
  - **ノルム異常 24 件** — `|q|` が 0.000122 / 1.106 / 2.1307 / **2.230613 (10 回、同じ値)**。
    正常なクォータニオンは常に `|q|`=1 で、運動しても変わらない。しかも**機体が静止して
    いる (gyro 0.00) ときにも出る**。同じ値が繰り返すのはビット単位の読み出し化けを示唆する。
  - **角速度スパイク 9 件** — 1906〜2034 deg/s。前後のサンプルは 0.2〜3.8 rad/s で、
    **孤立した 1 サンプルだけが飛んでいる** (20 ms で 0.28 → 35.51 → 0.28 rad/s は
    角加速度 1775 rad/s² に相当し、物理的にありえない)。姿勢は同時刻に動いていない。

#### そのうえで見つかった不具合 — 姿勢が飛ぶと復帰できなかった

t=6.04s に **姿勢基準そのものが 169° 飛び、飛んだ先で正常に追従を続ける**事象が出た
(`|q|`=1.000、以降なめらか、角速度からの予測はわずか 2.1°)。跳躍は「最後に *採用* した
値」との差分で見るため、比較対象が飛ぶ前の姿勢に固定され、**残り 144 秒をすべて棄却し
続けた** (棄却率 96%、連続棄却 7202 回)。姿勢が古い値に貼り付くので、制御は事実上盲目になる。

**修正**: `stale` (連続棄却が `stale_after` 超) に達したら**跳躍チェックだけを解除して
再同期する**。絶対値で判定できるノルム異常とフルスケール化けは弾き続ける。同じ bag で
再生すると棄却率 **96.0% → 0.52%**、連続棄却 **7202 → 6 回** (120 ms で復帰)。
`rl_attitude_node` は再同期時に警告を出す — **目標姿勢は飛ぶ前の基準で与えられているので、
飛んだあとは目標を入れ直す必要がある**。

> 残る問題は **IMU 側**。150 秒に 1 回とはいえ姿勢基準が飛ぶので、水中で起きると
> その瞬間に目標姿勢の意味が変わる。BNO055 のキャリブレーション状態
> (`CALIB_STAT` レジスタ) を確認・記録することを検討する。

再現と評価は記録した bag で手元でやり直せる:

```bash
./tools/record_run.sh --bag-only --name imu-motion     # 実機で録る
python3 tools/imu_sanity_replay.py <bag> --sweep       # PC で閾値を振って評価
```

### A-2. 【解決済み】`max_rate_hz` のキャップがエイリアシングを起こす

`perception_node` の間引きが「前回**処理した時刻**から一定時間空ける」方式だったため、
入力が上限より少しでも速いと**必ず 1 フレームおきに落ち、目標の半分近くまで下がって**いた。

| 入力 | 上限 | 修正前の実測 | 修正後 |
|---:|---:|---:|---:|
| 15 Hz | 10 Hz | **7.91 Hz** | **10.00 Hz** |
| 13.4 Hz | 10 Hz | **7.78 Hz** | 約 10 Hz |

**実装済み**: `umiusi_autonomy/rate_limiter.py` の `RateLimiter`。期限を「実際に通した時刻」
ではなく「前回の期限」から進めることで位相が入力に追従し、目標レートに最も近いフレームを
選ぶ。処理が詰まって期限を大きく過ぎた場合は、溜まった期限を消化せずその時点から張り直す
(詰まりが解けた瞬間のバースト防止)。

ROS 非依存なので単体テストできる。**実機で観測したレートの組み合わせを使ったテスト 11 件が
通っている** (`umiusi_autonomy/test/test_rate_limiter.py`)。修正前の挙動も再現テストとして
残してあるので、同じ退行を検知できる。

あわせて、**画像の `header.stamp` が設定されていない publisher だと stamp が 0 のまま進まず
全フレームが落ちて perception が沈黙する**という既存の弱点も塞いだ (その場合はノードの
時計に切り替え、一度だけ警告する)。

> **実機では未検証** (実装時に機体が停止していたため)。次回の実機作業で、
> `max_rate_hz` を 10 に戻したときに認識周期が 10 Hz 付近に張り付くことを確認すること。

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

### A-4. 【解決済み】バンドル済み RL ポリシーが実機で読めない

`models/cruise_policy/final.zip` は **numpy 2.5.0** で保存されており、Pi (ROS Jazzy 標準の
**numpy 1.26.4**) では `ModuleNotFoundError: No module named 'numpy._core.numeric'` で失敗する。
`custom_objects` でも `numpy._core` のシムでも回避できない。

**修正**: 重み + 正規化統計を素形式へ書き出し、SB3/cloudpickle 非依存の torch 推論に切り替える。
検証済みの実装が `umiusi_rl_control/policy_infer.py` にあり、**SB3 との出力差は 200 サンプルで 0.000e+00**。

- 書き出し: `umiusi_sim/export_policy.py` → `weights.pt` + `obs_norm.npz` + `meta.json`
- 実機側は **torch だけ**でよくなる (SB3・gymnasium・cloudpickle が不要になる)
- 副次効果として実機の依存が大幅に減る

**実装済み** — `rl_attitude_node._ensure_model()` が `export/` を自動採用する
(`_try_export_model()`)。実機で外部シム無しに動作することを確認済み。
`model_path` を明示した場合は**その隣の `export/` だけ**を見る (バンドル済みポリシーへ
黙って落ちると、新しいポリシーを試しているつもりで巡航ポリシーが動く事故になるため)。

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

**autonomy 側の回避策 (2026-08-21)**: `tools/umiusi_stack.sh` が同梱の
`umiusi_autonomy/config/cameras_deploy.yaml` (`/dev/video4`) を `cameras_param_file:=` で
自動的に渡すようにした。**手で `ros2 launch sinsei_umiusi_control main.yaml` する場合は
自分で渡す必要がある**。上流の `params/cameras.yaml` 自体は未修正なので、この項目は残す。

渡し忘れると `camera_bridge_node` が RTSP を開けず、
`ハードウェア経路 ... software に落とします` → `接続できません` を再接続間隔ごとに出し続ける
(警告自体は 10 秒 throttle 済み)。

### B-2. 【解決済み】前カメラ (CSI) と libcamera — apt 版を入れてはいけない

前方カメラは **Raspberry Pi Camera Module 3 NoIR (`imx708_noir`)**。unicam (`/dev/media1`) と
ISP (`/dev/media0`) には最初から正しく認識されている。

**公式 Wiki (raspi-setup-3) のとおり、apt の libcamera は Camera Module V3 に対応しておらず、
チームは `raspberrypi/libcamera` を `/usr/local` にソースビルドしている。**
`apt install gstreamer1.0-libcamera` を入れるとそちらが優先され、Pi 用 IPA を持たないため
次で失敗する:

```
WARN  IPAManager  No IPA found in '/usr/lib/aarch64-linux-gnu/libcamera'
ERROR RPI  Failed to load a suitable IPA library
ERROR RPI  Failed to register camera imx708_noir: -22
```

**対処: apt 版を入れない (入れてしまったら purge する)。** `/usr/local` 版を使うための
環境変数は公式手順で `~/.bashrc` に設定済み:

```bash
export LIBCAMERA_IPA_PROXY_PATH=/usr/local/libexec/libcamera
export LIBCAMERA_IPA_CONFIG_PATH=/usr/local/share/libcamera/ipa
export LD_LIBRARY_PATH=/usr/local/lib/aarch64-linux-gnu:/usr/local/lib:$LD_LIBRARY_PATH
export GST_PLUGIN_PATH=/usr/local/lib/aarch64-linux-gnu/gstreamer-1.0:$GST_PLUGIN_PATH
```

### B-2b. 【中】`.bashrc` の環境変数は非対話シェルに効かない

上記は `~/.bashrc` にあるが、`.bashrc` の先頭に
`case $- in *i*) ;; *) return;; esac` という**非対話なら即 return するガード**があるため、
systemd・スクリプト・非対話 SSH からの起動では**設定されない**。

人間が対話 SSH して `ros2 launch` する分には効くが、自動起動では効かない。
`tools/umiusi_stack.sh` と `tools/acceptance_test.sh` は自前で設定して補っている。

> この挙動を知らずに「`libcamerasrc` が無い」と誤診したことがある。非対話で確認するときは
> 環境変数の有無を先に疑うこと。

### B-3. 【中】カメラ 1 本の失敗でノードが落ちる

`gst_camera_node` はパイプラインエラー時に `rclcpp::shutdown()` を呼ぶ。片方のカメラが
使えないと、そのノードが丸ごと消える。再試行かデグレード起動があると運用が楽になる
(`sinsei_UMIUSI_control` 側の変更)。

### B-4. 【仕様】有線接続はインターネット共有 + mDNS が正規手順

公式 Wiki (raspi-setup-2) の「ネットワーク設定」がこの構成の根拠:

> `/etc/netplan/99-umiusi.yaml` … **PC からのインターネット共有の有無に関わらず、mDNS が
> 動くようにしたいという意図。**
> ```yaml
> eth0: { dhcp4: true, link-local: [ipv4], optional: true }
> ```

つまり:

* **接続は IP ではなく `ssh pi@<機体名>.local` (mDNS)** で行う。固定 IP は使わない
* **PC からのインターネット共有が正規のワークフロー** (Pi が apt / pip / git を使えるようにする)。
  Linux の PC なら `sudo nmcli con mod "<有線接続名>" ipv4.method shared`
* 共有が無くても `link-local: [ipv4]` により `169.254.x.x` を自己割り当てするので mDNS は効く。
  ただし**インターネットは使えない**
* **共有をオンオフした直後は、Pi が経路変更に適応するまで数分かかる** (Wiki に明記あり)

**ファイアウォールに注意。** Wiki も「接続できない場合はファイアウォールをオフにして試す」と
書いている。Linux の PC で `ufw` が有効だと **ROS 2 の DDS が通らず、トピックが一切見えない**:

```bash
sudo ufw allow in on <有線IF> from 10.42.0.0/24
```

SSH だけなら PC 側の ufw は関係ない (PC からの outbound のため)。**PC でも ROS 2 を動かして
Pi と通信する場合にのみ必要。**

### B-5. 【解決済み】`pip install torch` は aarch64 でも CUDA 版を引く

PyPI 既定の `torch` は aarch64 でも `+cu130` を取得し、`nvidia-*` 一式で **4.5 GB** を
消費する (GPU は無いので全て無駄)。さらに `setuptools` が 84 に上がり
`colcon-core` の制約 `<80` に違反してビルドが壊れる。

**対処**: 機体の `/etc/pip.conf` で CPU インデックスを既定にする。

```ini
[global]
break-system-packages = true
index-url = https://download.pytorch.org/whl/cpu
extra-index-url = https://pypi.org/simple
```

これで `pip install torch` も `rosdep install` 経由の pip も CPU 版を取る。
`torch` は rosdep のカスタムルール (`rosdep/umiusi.yaml` の `python3-torch-cpu`) から
`package.xml` に宣言済みなので、**`rosdep install` だけで入る**。

> Ubuntu 24.04 に `python3-torch` という apt パッケージは**存在しない** (universe を
> 有効にしても無い) ため、pip 経由になる。

### B-6. 【解決済み】実機に `pip` が入っていない + rosdep の PEP 668 拒否

Ubuntu 24.04 の Pi イメージには `python3-pip` も `ensurepip` も無い。加えて
**rosdep の pip インストーラは PEP 668 環境で実行自体を拒否する**
(`externally managed` として `InstallFailed` を投げる)。

**対処**: `/etc/pip.conf` に `break-system-packages = true` を書く
(`PIP_BREAK_SYSTEM_PACKAGES=1` でも可 — rosdep 自身が推奨している方法)。
pip 本体は `apt install python3-pip`、無ければ `get-pip.py` で入れる。
手順は `robot_setup.md` の 2-1。

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

1. **B-8** — CAN テレメトリ。ハードが繋がらないので最優先
   (A-1 / A-2 / A-3 / A-4 / B-1 / B-2 は解決済み)
3. **B-9** — セットアップの再現性 (B-4/B-5/B-6 は解決済み)
4. **B-7** — 性能
5. **A-5 / A-6 / B-3** — 運用性・分かりやすさ
