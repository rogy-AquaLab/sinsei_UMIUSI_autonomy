# 10/18 競技前 手順書

上から順に進める。各ステップの「確認」が通らなければ、最後の「うまくいかないとき」の表で戻す。
core の `scripts/field/*.sh` は使わない。端末は `~/.bashrc` が読まれる普通のログイン（ROS・libcamera の環境はそこで入る）。

**ブランチ**: core / control / ui は `competition/20261018`、msgs は `develop`。
competition には develop 向けの PR 7 本（core #97 #98 #99 #101、control #346、ui #75 #76）に加えて、次が入っている。

- core: 現場の既定値（`aim_y: 0.3`、`use_hold_yaw: true`、`cnn_model: balloon_cnn_20261015.onnx`）

1 基が壊れたとき用の 3 基モードは別ブランチ `competition/20261018-3thruster`（control / ui）。壊れるまで使わない（表の「1 基が回らない」）。

現場で直したものは `competition/20261018` に push し、Pi で `git pull`（control は `colcon build` も）。3thruster 側へは `git merge competition/20261018` で取り込む。

## 1. Pi を competition/20261018 に更新（約 10 分、プールに入る前）

戻すときのために、今の commit を控えてから更新する。`!!!` が出たらそこで止める。

```bash
cd ~/ros2-ws/src
for r in sinsei_*_[cC][oO][rR][eE] sinsei_*_[cC][oO][nN][tT][rR][oO][lL] sinsei_*_[mM][sS][gG][sS]; do echo "$r $(git -C $r rev-parse --abbrev-ref HEAD) $(git -C $r rev-parse --short HEAD)"; git -C $r status --short; git -C $r diff > ~/before_1018_$r.diff; done | tee ~/before_1018.txt
# ↑ ファイル名が出た repo は git -C <dir> checkout -- <file> で片付ける（残っていると切り替えに失敗する）。
#   中身は ~/before_1018_<dir>.diff に控えてある（core には yk4to の現場調整の yaml が残っているはず）
for r in sinsei_*_[cC][oO][rR][eE] sinsei_*_[cC][oO][nN][tT][rR][oO][lL]; do git -C $r fetch && git -C $r checkout competition/20261018 && git -C $r pull --ff-only || { echo "!!! $r で失敗"; break; }; done
git -C sinsei_*_[mM][sS][gG][sS] fetch && git -C sinsei_*_[mM][sS][gG][sS] checkout develop && git -C sinsei_*_[mM][sS][gG][sS] pull --ff-only || echo "!!! msgs で失敗"
cd ~/ros2-ws && colcon build   # いつもの引数で
for p in core control; do ros2 pkg prefix sinsei_umiusi_$p; done   # 両方 ~/ros2-ws/install/… であること
```

`ros2 pkg prefix` に `ws-claude` が出たら、`~/.bashrc` の `ws-claude` を source している行を消して端末を開き直す（10/15 の古い core が上に重なって、competition が使われない）。

**確認**（control を起動する前。カメラが空いているうちに）: 前カメラの設定がこの Pi で通るか。`OK` だけが出ること。

```bash
timeout -s INT 5 gst-launch-1.0 libcamerasrc af-mode=manual lens-position=1.0 awb-enable=false colour-gains="<2.7,1.45>" ! video/x-raw,width=1280,height=720 ! fakesink 2>&1 | grep -iE 'error|no property' || echo OK
```

`OK` 以外が出たら、表の 1 行目のとおり yaml からホワイトバランスを消してから起動する（そのまま起動すると前カメラが止まり続ける）。

UI も `competition/20261018` の最新にする（いつもの方法で）。

## 2. 起動と確認

tmux でウィンドウごとに順に起動する。

1. 偽装（2 つ）
   - `ros2 topic pub /high_power_health_check_result sinsei_umiusi_msgs/msg/HealthCheckResult "{is_ok: true}"`（ずっと出す）
   - `ros2 topic pub /state/low_power_circuit_info sinsei_umiusi_msgs/msg/LowPowerCircuitInfo "{can: 0, headlights: 0, imu: 0, indicator_led: 0}"`（**control を起動したら Ctrl-C**。両方出ると UI の値が混ざる）
2. core: `export PYTHONPATH=~/ortlib:~/ws-claude/ortlib:$PYTHONPATH` → `ros2 launch sinsei_umiusi_core main.yaml`
   - export は CNN を onnxruntime で回すため（10/15 に入れたのは `~/ws-claude/ortlib`。無い方は無視される）。`detector: hsv`（既定）なら CNN は使わないので、無くても走りは変わらない
   - CNN を使うことになったら、core のログの「CNN を読んだ: …（onnxruntime、…）」を見る。`opencv` なら export を忘れている（core を起動し直す）
3. UI で Connect → Main Power On
4. control: `ros2 launch sinsei_umiusi_control main.yaml`（引数なし。共有メモリを出す cameras_shm.yaml が既定）
5. 記録（走る前に始め、上がってから止める。止めるのは各ウィンドウで Ctrl-C。kill -9 は使わない。10/15 と同じもの）
   - bag: `mkdir -p ~/rosbags && ros2 bag record -o ~/rosbags/$(date +%Y%m%d_%H%M%S) /cmd/target /cmd/attitude_target /robot_state /auto_target_generator/status /client_count /state/imu /state/low_power_circuit_info /state/thruster_state_all /rosout`
   - 映像（前・下、H.264 のまま）: `UMIUSI_REC_DIR=~/recordings/$(date +%Y%m%d_%H%M%S) ~/ros2-ws/src/sinsei_*_[aA][uU][tT][oO][nN][oO][mM][yY]/tools/record_camera.sh --both`

**確認**

- [ ] UI に前カメラの映像が出る
- [ ] UI の Auto タブで「読み直す」→ 全部に値が出る（「この core に無い」が無い）。`aim_y` が 0.3、「Auto 以外でも認識を表示」が on
- [ ] `ros2 topic echo --once /auto_target_generator/status` に `"new_frame": true` が出る（自律が映像を読めている）
- [ ] MANUAL で少し動かして、スラスタが 4 基とも回る

## 3. テザーで沈めてカメラを確認

ホワイトバランスは既定で固定（赤 2.7、青 1.45）、ピントは 1 m に固定。風船を前に置いて UI の映像を 1〜2 分見る。

- [ ] 真っ青（赤い風船が黒い）に飛ばない（10/17 は自動で数秒ごとに飛んだ）
- [ ] 黄・赤の風船がその色に見える
- [ ] 1〜2 m の風船にピントが合う

**調整するとき**（control は止めない。前カメラだけ 2 秒で上がり直す）

```bash
P=$(ros2 pkg prefix sinsei_umiusi_control)/share/sinsei_umiusi_control/params/cameras_shm.yaml
cp -n $P ~/cameras_shm.yaml.bak   # 元の控え（2 回目以降は上書きしない）
nano $P                          # libcamerasrc の行を直す
pkill -f '[_]_node:=pi_camera'   # 前カメラが新しい値で上がり直す（-9 は付けない）
```

| 見え方 | 直すところ（`colour-gains="<赤,青>"`、`lens-position`） |
| --- | --- |
| 全体が緑・赤が暗い | 赤を上げる（例 `<3.0,1.3>`、`<3.5,1.15>`） |
| 全体が青い | 青を下げる（例 `<2.7,1.2>`） |
| 全体が赤い・ザラつく | 赤を下げる（例 `<2.3,1.6>`） |
| 近くがぼける / 遠くがぼける | `lens-position` = 1 / 距離[m]。例 2.0 = 50 cm、0.5 = 2 m |

- 値は風船の色で決める。完全に白くしなくてよい（安定していれば次の色合わせで吸収できる）。値を変えたら色合わせはやり直し
- 決まったら `cp $P ~/cameras_shm.yaml.good`。`colcon build` で戻ったら `cp ~/cameras_shm.yaml.good $P` → `pkill -f '[_]_node:=pi_camera'`
- build が `--symlink-install` だと、nano で src の yaml も変わる。後で `git pull` するときは先に `git -C <control> checkout -- params/` で戻す（値は .good に残っている）

## 4. 色合わせと認識の確認

カメラの値を決めた後にやる。control は動かしたまま。

```bash
export PYTHONPATH=~/ortlib:~/ws-claude/ortlib:$PYTHONPATH
python3 ~/ros2-ws/src/sinsei_*_[cC][oO][rR][eE]/scripts/capture_frames.py --seconds 30   # 風船を近く・遠く・暗い所と色々写す
python3 ~/ros2-ws/src/sinsei_*_[cC][oO][rR][eE]/scripts/calibrate_server.py --port 8766  # http://192.168.50.1:8766/
```

- 画面の「自動で合わせる」（CNN が先生）→ 黄・赤を確かめて保存。保存すると 1 秒以内に自律が読み直す
- 一致度 (f1) が 0.5 未満の色は採られない（元のまま）。保存前の設定は `~/umiusi_balloon/history/` に残る

**確認**

- [ ] 黄の風船を 1〜3 m で見せて、status の `detections` に yellow が出続ける
- [ ] 風船が無い方向（壁・水面・底）で何も出ない
- [ ] 赤も同じ（2 走目に使う）

## 5. 出走

UI の Auto タブで値を変えて「反映」。次に AUTO に入ったときから効く。機体の再起動や自律ノードの再起動（落ちると自動で上がり直す）で yaml の値に戻るので、**「読み直す」での確認は AUTO に入る直前に**やる。

| 項目 | 1 走目（黄だけ） | 2 走目（既定） |
| --- | --- | --- |
| 狙う色 `mission.priority` | yellow | red,yellow |
| 最初に狙う色 `mission.first_color` | yellow | yellow |
| 同じ色に続けて突っ込む上限 `mission.max_rams` | 99 | 3 |

1. 「読み直す」で上の値になったことを確かめる
2. UI: Back to Standby → Operation Mode を AUTO に → Enter Operation（MANUAL から直接は入れない。AUTO がスタートの合図で、`mission.start_delay` 後に動き出す）
3. AUTO の間は無線が切れても電源は落ちない。上がったら UI が自動でつなぎ直す。もう一度走るときも Back to Standby → Enter Operation

## うまくいかないとき

カメラの yaml は 3. の `$P`。直したら `pkill -f '[_]_node:=pi_camera'`。

| 症状 | まず | それでもだめなら |
| --- | --- | --- |
| 1. の確認で `OK` が出ない、または control 起動後に UI の前カメラが映らない | `$P` と同じ場所の `cameras.yaml` の両方から ` awb-enable=false colour-gains="<2.7,1.45>"` を消す（自動に戻る） | ` lens-position=1.0` も消して `af-mode=continuous` に。それでも映らなければ下の「全部戻す」 |
| pkill の後、前カメラが戻らない | `rm /tmp/umiusi_cam1.sock` → もう一度 pkill | control を再起動 |
| 固定しても色が真っ青に飛ぶ | 値が効いていない。1. の確認をやり直す | 自動に戻し、1 走目は黄だけで走る |
| status に `"new_frame": true` が出ない（自律が映像を読めない） | control のログの `Starting GStreamer pipeline` に `shmsink` があるか見る | control を `cameras_param_file:=$(ros2 pkg prefix sinsei_umiusi_control)/share/sinsei_umiusi_control/params/cameras_shm.yaml` 付きで起動し直す |
| Auto タブで読めない・反映できない | 「読み直す」。UI を開き直す | `ros2 param set /auto_target_generator mission.priority "[yellow]"`、`ros2 param set /auto_target_generator mission.max_rams 99`（2 走目は `"[red, yellow]"`、`3`） |
| 色合わせを保存したら悪くなった | `~/umiusi_balloon/history/` の一番新しいものを `~/umiusi_balloon/current.json` へ cp | — |
| Main Power On が low power で断られる（UI が切れて電源が落ちた後など） | 2. の low_power の偽装をもう一度出して押す → 通ったら Ctrl-C | — |
| 1 基が回らない・壊れた | control を止めて `cd ~/ros2-ws/src && git -C sinsei_*_[cC][oO][nN][tT][rR][oO][lL] checkout competition/20261018-3thruster && git -C sinsei_*_[cC][oO][nN][tT][rR][oO][lL] pull --ff-only` → `colcon build` → control を起動。UI も `competition/20261018-3thruster` にして Connect → サイドバーの Thrusters で止める 1 基を選ぶ（その場で 3 基。「4 基」で戻る） | UI に Thrusters が出ない・選べない: `ros2 param set /attitude_controller mixer.disabled_thruster <lf\|lb\|rb\|rf>` |
| 更新後に全体がおかしい（全部戻す） | `~/before_1018.txt` の 3 つを `git -C <dir> checkout <commit>` → `colcon build`。control は `cameras_param_file:=$(ros2 pkg prefix sinsei_umiusi_control)/share/sinsei_umiusi_control/params/cameras_shm.yaml` 付きで起動（前の既定は共有メモリを出さない）。UI も前の版に | — |
