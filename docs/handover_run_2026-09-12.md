# 実機を回す手順 — 2026-09-12

**このリポジトリを知らない人が、この 1 枚だけで回せるように書いてある。**
上から順にやれば終わる。コマンドはコピペで動く。**迷ったら止めてよい** — 録り直しは安い。

- **やること**: ① 古典制御と RL をそれぞれ動かして比較する ② 余裕があれば duty の掃引
- **やらないこと**: カメラの実験 / ゲインの調整 / 解析（持ち帰る）
- **かかる時間**: ① 約 40 分 / ② 約 25 分

**バグが出たらその場で直す。解析は持ち帰る。** 今日は「動くか」と「どちらが良いか」の
感触を取るのが目的。数値の詰めは後日。

---

# 先に読む①: 慌てなくていいもの

**以下は既知の未実装によるもので、故障ではない。** この日に直さない。

| 見えるもの | 正体 |
|---|---|
| `Failed to write Can: Not implemented for ...` が 3 秒おきに延々と出る | CAN 書き込みの一部が未実装。**正常** |
| `high_power_circuit_info` の `voltage` / `current` / `temperature` が `0.0` | メイン電源基板の読み出しが未実装。**ESC 個別の電圧が本物** |
| `low_power_circuit_info` の `headlights` と `indicator_led` が `1` (ERROR) | ソフト側の既知の不具合。ライトの故障ではない |
| IMU のログに `previous async trigger is still in progress` が数分に数回 | **正常** |
| サーボが小さく唸る | CAN の指令レートが低いため。**測定には影響しない** |

# 先に読む②: 直ちに止めるもの

- `water_leaked` が `true` になった
- ESC の電圧が **20 V を下回った**
- 異音・異臭・発熱
- 1 基だけ極端に挙動が違う
- **判断に迷った**

**止め方**（どちらでもよい。速いほうを使う）:

```bash
# 走っているノードの窓で Ctrl-C   -> ゼロ出力と disarm を自分で送って終わる
# または別の窓から e-stop:
ros2 topic pub --once /classical_attitude/estop std_msgs/msg/Bool '{data: true}'   # 古典
ros2 topic pub --once /rl_attitude/estop        std_msgs/msg/Bool '{data: true}'   # RL
```

そのあと運用担当に連絡する。

---

# 手順

## 1. 機体に入る

```bash
ssh pi@umiusi2.local
```

**到達経路（踏み台・パスワード）は運用担当に確認すること。** ここには書けない。

## 2. tmux を開く

```bash
tmux new -s run
```

- 新しい窓: `Ctrl-b` → `c`
- 窓の切り替え: `Ctrl-b` → `0` / `1` / `2` ...

> **必ず tmux の中で動かす。** ssh が切れるとスタックが黙って死ぬ。

## 3. 何が入っているか記録する

```bash
cd ~/ros2-ws
source install/setup.bash
git -C src/sinsei_umiusi_control log --oneline -1
```

出た行を**手元にメモする**。あとで「どのコードで録った bag か」が分からなくなる。

> **ビルドし直さないこと。** 入っているものをそのまま使う。

## 4. CAN と ESC が生きているか（スタックを上げる前）

```bash
ip -br link show can0
```

- **期待値**: `can0  UP`
- **ダメなとき**: `DOWN` なら止めて連絡

```bash
timeout 5 candump can0 | head -5
```

- **期待値**: 行が流れる。`0000097C` のような 8 桁の ID が見える
- **ダメなとき**: 何も流れないなら ESC が生きていない。止めて連絡

## 5. スタックを上げる（窓 0）

```bash
cd ~/ros2-ws/src/sinsei_UMIUSI_autonomy/tools
./umiusi_stack.sh start --control-only
```

`不明な引数: --control-only` と出たときは機体のコードが古い。代わりにこれを叩く
（どのバージョンでも動く。**機体のコードは更新しない**）:

```bash
source ~/ros2-ws/install/setup.bash
ros2 launch sinsei_umiusi_control main.yaml enable_cameras:=false
```

> 他のモード（`start` / `--attitude` / `--perception`）は**使わない**。
> 指令を出すノードが上がると、これから自分で上げるノードと衝突する。

30 秒待つ。

## 6. 上がったか確認する（新しい窓）

```bash
source ~/ros2-ws/install/setup.bash
ros2 control list_controllers
```

- **期待値**: **6 個すべてが `active`**

  ```
  thruster_controller_lf / lb / rb / rf    active
  attitude_controller                      active
  gate_controller                          active
  ```

- **ダメなとき**: 1 つでも `active` 以外なら止めて連絡

```bash
for t in /state/imu /state/thruster_state_all /state/high_power_circuit_info; do
  echo "--- $t"; timeout 8 ros2 topic hz -w 50 $t 2>&1 | tail -2
done
```

- **期待値**: どれも `average rate: 49.9`〜`50.1`

```bash
ros2 topic echo --once /state/high_power_circuit_info
```

- **期待値**: `esc_*_state.voltage` が **4 つとも 22〜25 V**、`water_leaked` が全部 `false`
- **ダメなとき**: 20 V 未満なら充電されていない。`water_leaked` が `true` なら直ちに止める

## 7. 古典制御の前提を入れる

古典制御は `umiusi_perception` の中の制御ライブラリを呼ぶ。**入っていなければ入れる。**

```bash
python3 -c "from umiusi_perception.classical import ClassicalController; print('OK')"
```

- **`OK` と出たら** 手順 8 へ
- **`ModuleNotFoundError` / `ImportError` が出たら** 入れる:

  ```bash
  pip install --no-deps --no-index ~/umiusi_sim/packages/perception
  ```

  `--no-deps` が要る。**機体はインターネットに出られない**ので、これを省くと依存の
  解決に行って失敗する。制御だけなら numpy しか使わないので依存は既に足りている。

  `~/umiusi_sim` が無ければ**このステップは飛ばし、古典はやらずに RL だけ**やる
  （手順 10）。運用担当に連絡しておく。

## 8. 機体を固定する

**係留するか、手で押さえる。**

姿勢制御の比較なので、**機体が傾けられる程度に自由**であってほしい。完全に固定すると
姿勢が動かず比較にならない。**流されない程度に緩く係留する**のが理想。

壁から十分離す。**どちらの制御も推力を出すので機体は動く。**

## 9. 記録を開始する（新しい窓）

**駆動より先に開始する。比較する 2 本は同じ bag に入れてよい**（あとで時刻で分けられる）。

```bash
cd ~/ros2-ws/src/sinsei_UMIUSI_autonomy/tools
./record_run.sh --bag-only --name 20260912-classical-vs-rl
```

20 秒後に「何を購読できたか」が出る。

- **期待値**: 一覧に **`/state/imu`** と **`/state/thruster_state_all`** がある
- **ダメなとき**: 無ければ録れていない。`Ctrl-C` して手順 5 からやり直す

## 10. 古典制御を動かす（新しい窓）

**まず指令を出さずに計算だけさせて、落ちないことを見る:**

```bash
source ~/ros2-ws/install/setup.bash
ros2 launch umiusi_autonomy classical_attitude.launch.py publish:=false
```

- **期待値**: 次の 3 行が出て、そのまま生き続ける

  ```
  bundle: .../classical_bundle.json (cap_ref=0.25, 50 Hz, 到達速度 @max_duty=0.208 m/s)
  arm state: DISARMED (e-stop on '~/estop', arm service '~/arm')
  classical attitude: 20 ms, max_duty=0.25, publish=False
  ```

- **`未較正の契約値: ...` の警告が 5 行出るのは正常。** 絶対値が未較正という表示
- **`umiusi_perception.classical を import できません` なら** 手順 7 に戻る
- **`バンドルがありません` なら** 機体のコードが古い。古典は諦めて RL だけやる

`Ctrl-C` で止める。次に**本番（指令を出す）**:

```bash
ros2 launch umiusi_autonomy classical_attitude.launch.py
```

起動しただけでは動かない（`DISARMED`）。別の窓で **arm する**:

```bash
ros2 service call /classical_attitude/arm std_srvs/srv/SetBool '{data: true}'
```

- **期待値**: `success=True, message='armed'`

指令が出ているか見る:

```bash
ros2 topic echo --once /cmd/direct/thruster_controller/output_lf
```

- **期待値**: `runnable: esc: true / servo: true`、`duty_cycle` が 0 でない
  （何もしなくても **0.15 前後**出る。浮力に抗して深度を保つため。正常）
- **`angle` が ±88 度付近になるのも正常**。推力を下に向けるためサーボをほぼ振り切る

**2〜3 分そのまま観察する。** 見るもの:

- 機体が**姿勢を保とうとするか**（手で傾けて戻るか）
- **発振しないか**（行き過ぎて戻り、を繰り返さないか）
- サーボが**振り切ったまま張り付かないか**

終わったら **arm を切る**:

```bash
ros2 service call /classical_attitude/arm std_srvs/srv/SetBool '{data: false}'
```

そのあと launch の窓で `Ctrl-C`。

## 11. RL を動かす（同じ手順で）

**古典のノードを完全に止めてから。** 同時に上げると `/cmd/direct` を取り合う。

```bash
source ~/ros2-ws/install/setup.bash
ros2 launch umiusi_rl_control rl_attitude.launch.py publish:=false   # まず計算だけ
```

落ちないことを確認したら `Ctrl-C`。本番:

```bash
ros2 launch umiusi_rl_control rl_attitude.launch.py
ros2 service call /rl_attitude/arm std_srvs/srv/SetBool '{data: true}'   # 別の窓
```

**古典と同じ 2〜3 分、同じ項目を見る。** 条件を揃えるのが目的なので、
**手で傾ける強さと向きも似せる**。

> **RL は指令ゼロでも duty が上限（0.25）に張り付く。** これは既知で、方策が横方向の
> 速度を観測できないことによる。**故障ではない。** 古典との差として記録するだけでよい。

終わったら arm を切って `Ctrl-C`。

## 12. 止めて回収する

記録の窓で `Ctrl-C`。bag が閉じる。

```bash
ls -la ~/runs/latest/ && du -sh ~/runs/latest/
```

**手元の PC から**（機体の中からではない）:

```bash
scp -r pi@umiusi2.local:runs/latest/ ./20260912-classical-vs-rl/
```

**録っただけでは終わっていない。** 8 月に 1 回、記録が機体の再起動で失われている。

## 13. 記録に残す

**これが今日の成果物。** 数値の解析は後日やるので、**見た印象をそのまま書く**のが役に立つ。

- [ ] 手順 3 でメモした `git log` の行
- [ ] **古典を動かせたか**（手順 7 で入れたか / 元から入っていたか / 諦めたか）
- [ ] **どちらがどう見えたか** — 姿勢の保ち方 / 発振の有無 / サーボの張り付き / 音
- [ ] **arm した時刻**（古典と RL それぞれ）。bag を時刻で切り分けるのに使う
- [ ] 出たエラーは**文面をそのまま**（要約しない）
- [ ] 途中で止めた場合はその理由
- [ ] 機体をどう固定したか、水温・水深など分かる範囲

---

# 補足

## なぜ duty と回転数を測るのか

いま **「指令した duty で実際に何ニュートン出ているか」が分かっていない**。
モデルの指数が 2.0 なのか 1.0 なのかで、到達速度の予測が 2 倍以上変わる。

`/state/thruster_state_all` には**回転数が実測で入る**。推力は回転数の 2 乗に比例する
（これは物理として確か）ので、**duty と回転数の関係が分かれば指数が決まる**。
秤も治具も要らない。**必要なのは「duty を段階的に振って回した bag」だけ。**

過去の bag はスラスタを回していない状態で録られていて使えなかった。
**今回は必ず回すこと。**

## 回収した bag の見方（担当者向け・現場では不要）

```bash
python3 tools/duty_rpm_fit.py <bag-dir>
```

`rpm = a * duty` の当てはめと決定係数を出す。線形に近ければ指数は 2.0 に近く、
飽和していれば 2.0 未満。

スラスタを回していない bag には「Runnable のサンプルが無い」と報告して終わる。
**回し忘れがこれで分かる。**

## 関連

| ファイル | 中身 |
|---|---|
| `thrust_calibration.md` | なぜ duty と回転数を測るのか、経路ごとの推力仮定の食い違い |
| `experiment_guide.md` | 較正実験の全体像（この 1 枚はその一部） |
| `known_issues.md` | 「慌てなくていいもの」の出どころ（A-1 / B-8） |
| `robot_setup.md` | 機体に入れない / `can0` が上がらないときの復旧 |

## 付録: duty の掃引（時間が余ったら）

推力モデルの同定用。**姿勢制御の比較とは別の実験**なので、比較が終わってからやる。

**このときは機体を固く固定する**（手順 8 の「緩く」とは逆）。プロペラの回転数は流入速度で
変わるので、機体が進むと `duty -> rpm` の関係が汚染される。

制御ノードは**全部止めてから**、1 基ずつ:

```bash
cd ~/ros2-ws/src/sinsei_UMIUSI_autonomy/tools
./record_run.sh --bag-only --name 20260912-duty-sweep     # 別の bag にする
./thruster_cmd.py sweep --ch lf --points 0.05 0.10 0.15 0.20 0.25 --dwell 12 --rest 3
```

`--ch` を `lb` / `rb` / `rf` に変えて 4 回。**1 基あたり約 2.5 分**（指定した 5 点が
正転と逆転の両方回るので 10 段 × 15 秒）。

> - プロンプトに「秤の読みを記録」と出るが、**この日は秤を使わない。** 無視してよい
> - **`--allow-full` は付けない。** duty 0.4 を超えられない安全装置が効く
> - **`--dwell 12` を省略しない。** 既定の 5 秒では立ち上がりの影響が抜けない

回した基の回転数を別の窓で確認（`^lf:` は基に合わせて変える）:

```bash
ros2 topic echo --once /state/thruster_state_all | grep -A4 "^lf:"
```

`rpm` が 0 のまま duty だけ上がるなら、そのスラスタは回っていない。低い duty で数秒
回してから再試行し、それでも 0 なら**メモして先へ進む**（無理に上げない）。
