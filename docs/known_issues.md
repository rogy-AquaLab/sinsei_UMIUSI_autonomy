# 既知の問題と修正方針

**いま残っている問題だけを書く。** 解決したものは項目ごと削除しているので、番号は飛ぶ
(識別子として安定させるため振り直さない)。優先度は **実機を動かすのにどれだけ効くか**。

---

## A. autonomy 側

### A-1. 【観測中】IMU のデータ化け — 検出はするが、いまは弾いていない

`/state/imu` に物理的にありえないサンプルが混入する。実機 (BNO055) で確認したもの:

* **ノルムが 0 のクォータニオン** — 静止 60 秒で 2 件 (約 30 秒に 1 回)
* **角速度が 3 軸とも ±35.6 rad/s** — int16 フルスケール (32767/16 = 2047.9 deg/s) と一致
* **0.5 秒で −3° → −170° → −4° の姿勢跳躍** — 運動中に出やすい

`navigator_node` / `auto_target_generator` / `rl_attitude_node` はいずれも角速度を
ヨーレートとして、姿勢をそのまま制御・観測に使うため、**1 発のスパイクで制御が跳ねる**。

**実装済み**: `umiusi_rl_control/imu_sanity.py` の `ImuSanity`。3 ノードすべての IMU
コールバックに入れてある。

> **2026-08-21 の方針変更 — 既定では捨てない (`imu_sanity_enforce: false`)。**
> 実機で測ったところ、判定に引っかかるのは **0.44%** しかなく（下記）、一方で
> **フィルタ自身が誤爆したときの被害のほうが大きい**ことが分かった（姿勢基準が飛んだあと
> 144 秒間ずっと棄却し続けた）。閾値を決めるにはまずデータが要るので、いまは
> **検出とログだけ行い、値はそのまま通す**。`imu_sanity_enforce:=true` で従来どおり捨てる。
>
> 唯一の例外は **ノルムが 0 のクォータニオン**。これは閾値の問題ではなく正規化そのものが
> 定義できない（0 除算）ので、`enforce` に関係なく直前の有効値を返す。
>
> ログは「破棄した」のか「検出したが通した」のかを言い分ける（`ImuSanity.describe`）。
> **データを集めてから閾値と運用を決め直すこと。**

| パラメータ | 既定 | 意味 |
|---|---:|---|
| `imu_sanity_enforce` | **false** | true にすると検出したサンプルを破棄する。既定は検出のみ |
| `imu_max_gyro` | 10.0 rad/s | 検出の閾値 (フルスケールは 35.74) |
| `imu_max_step_deg` | 30.0 deg | 1 サンプルの姿勢跳躍上限。50 Hz なら 1500 deg/s 相当 |

ROS 非依存の純関数なので単体テストできる。**実機で観測した実際の化け値を使ったテスト**が
`umiusi_rl_control/test/test_imu_sanity.py` にある。
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

**修正** (`enforce=true` にしたときのために残してある): `stale` (連続棄却が `stale_after`
超) に達したら**跳躍チェックだけを解除して再同期する**。絶対値で判定できるノルム異常とフルスケール化けは弾き続ける。同じ bag で
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

### A-9. 【中】`vel_cmd` を 0 にすると出力が飽和する（学習分布の外）

`vel_cmd` は `_build_obs()` で**観測ベクトルにそのまま入る**。巡航ポリシーは
**0.4 m/s で前進している状態しか学習していない**ので、0 を入れると分布外入力になり、
出力が飽和して `action = ±1` → **サーボが ±90° に張り付く**（実機で確認、2026-08-21）。

安全のために既定を 0 にしたが逆効果だったので **0.4 に戻した**。起動時の安全は
`start_armed=false`（arm するまで何も出さない）側で担保する。

> 姿勢だけ変えて前進を止めたい場合は、`AttitudeTarget` の `IGNORE_VELOCITY` で
> **速度指令に触らない**のが正しい（`set_attitude.py` は `--vel` 未指定でそうなる）。
> 「0 を送る」と「触らない」は別物。

### A-7. 【中】e-stop は `ros2 topic pub` の既定 QoS では届かない

`~/estop` の購読側は **TRANSIENT_LOCAL**（再起動時に e-stop 状態を継承するため latch する）。
一方 `ros2 topic pub` の既定は **VOLATILE** で、**Durability が合わずマッチしない**。
`Waiting for at least 1 matching subscription(s)...` が出続けて、**緊急停止が届かない**。
購読側は `ros2 topic info -v` で確認できる（Subscription count は 1 なのに繋がらない）。

```bash
ros2 topic pub --once --qos-durability transient_local \
    /rl_attitude_node/estop std_msgs/msg/Bool "{data: true}"
```

**回すときは手打ちに頼らず `teleop_keyboard` を開いておくこと**（`ESTOP_QOS` を使うので
QoS が確実に合う）:

```bash
ros2 run umiusi_rl_control teleop_keyboard
```

> 実機で踏んだ (2026-08-21)。ドキュメント側が QoS 指定なしのコマンドを載せていたのが原因で、
> 修正済み。**緊急停止の経路は、使う前に必ず「実際に止まるか」を確認すること。**

### A-8. 【解決済みだが再発しやすい】`rclpy.Node` の属性をメソッド名で潰さない

`teleop_keyboard` が `def handle(self, key)` を定義していて、`rclpy.Node.handle`
(プロパティ) を隠していた。`Node.__init__` の中の `with self.handle:` が

```
TypeError: 'method' object does not support the context manager protocol
```

で落ち、**ノードが起動すらできない**。エラーは rclpy の内部で出るので原因が見えにくい。
`handle_key` にリネームして解決 (2026-08-21、実機で踏んだ)。

`destroy_node` のように **`super()` を呼ぶ意図的なオーバーライドは問題ない**。
危ないのは Node の属性を「別の意味で」使ってしまうケース。新しいノードを書くときは
`handle` / `context` / `executor` / `clock` / `parameters` / `publishers` /
`subscriptions` / `timers` などを自分のメソッド名にしないこと。

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

水に入れる前に必須:

1. **B-8** — CAN テレメトリが取れず**浸水検知が効かない**。上流の未実装
2. **B-9** — `sinsei_umiusi_control` が古く、ESC の推力符号が逆。姿勢制御が発散する
3. **B-1** — `cameras.yaml` のデバイス指定。`umiusi_stack.sh` は回避するが上流は未修正

そのあと:

4. **A-1** — IMU の化けと姿勢基準の飛び。データを集めて閾値と運用を決める
5. **B-7** — `ros2_control` のリアルタイム優先度
6. **A-5 / A-6 / B-2b / B-3** — 運用性・分かりやすさ
