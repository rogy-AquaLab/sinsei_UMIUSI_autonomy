# 引き継ぎ用の実行手順 — 2026-09-12

**このリポジトリを知らない人が、単独でこの 1 枚だけを見て実機を回せることを目的にしている。**
コマンドはコピペして動く形で書いてある。期待値も併記したので、違ったら止めてよい。

この日の担当者は機体に行けないので、**判断が要る場面を作らない**ように手順を絞ってある。

## この日にやること (優先順)

| # | 内容 | 所要 | なぜ要るか |
|---|---:|---|---|
| 1 | 立ち上げと健全性チェック | 10 分 | 以下の前提。ここで異常なら 2 以降はやらない |
| 2 | **走行 bag の収録** | 20 分 | **これが本命。** いま推力モデルが未確定で、この bag が唯一の入口 |
| 3 | bag の回収 | 5 分 | 録っても取り出せないと意味が無い。過去に 1 回失われている |

## この日にやらないこと

- **古典制御 (`umiusi_perception.classical`) は動かさない。** 実装が実機未検証で、
  担当者が不在の日に初投入するものではない。
- **カメラを使う実験はしない** (1 の確認だけ)。CPU を食って 2 の記録に影響する。
- **パラメータの探索や調整はしない。** 既定値のまま録る。

---

## 0. 準備

```bash
# 手元の PC から機体に入る (経路は運用担当に確認すること)
ssh pi@umiusi2.local
```

**リモートで直接 `ros2 launch` を叩かないこと。** ssh が切れるとスタックが黙って死ぬ。
必ず `tmux` の中で動かす:

```bash
tmux new -s run          # 既にあるなら tmux attach -t run
# 窓の切り替え: Ctrl-b 0 / Ctrl-b 1 ... 新しい窓: Ctrl-b c
```

ビルド済みのものを使う。**この日にビルドし直さないこと**:

```bash
cd ~/ros2-ws
source install/setup.bash
git -C src/sinsei_umiusi_control log --oneline -1     # 何を焼いてあるかを記録に残す
```

出た行を**手元にメモしておく** (あとで「どのコードで録った bag か」が分からなくなる)。

## 1. 立ち上げと健全性チェック

### 1-1. CAN と IMU が生きているか (スタックを上げる前)

```bash
ip -br link show can0
```
期待値: `can0  UP` と出る。`DOWN` なら**ここで止めて運用担当に連絡**。

```bash
timeout 5 candump can0 | head -5
```
期待値: 行が流れる。`0000097C` のような 8 桁の ID が見える。
**何も流れないなら ESC が生きていない** — 止めて連絡。

### 1-2. スタックを上げる

**control 単体だけを上げる。** 姿勢制御 (RL) のノードは上げない —
この日の駆動に使う `thruster_cmd.py` は `/cmd/direct` に publish するので、
**姿勢制御ノードと同じトピックを取り合って両方おかしくなる**
(`thruster_cmd.py` の冒頭にも「rl_attitude_node と同時に動かさないこと」とある)。

窓 0 で:

```bash
cd ~/ros2-ws/src/sinsei_UMIUSI_autonomy/tools
./umiusi_stack.sh start --control-only
```

`--control-only` は**指令を出すノードを一切上げない**入口で、この用途のために用意してある。
カメラも UI も上がらない。

> 他のモード (`start` / `--attitude` / `--perception`) は**使わない**。
> 姿勢制御ノードが上がると `/cmd/direct` を取り合う。

**`不明な引数: --control-only` と出た場合**、機体のコードがこの入口より古い。
その場合は control を直接上げれば同じことになる (どのバージョンでも動く):

```bash
source ~/ros2-ws/install/setup.bash
ros2 launch sinsei_umiusi_control main.yaml enable_cameras:=false
```

**この代替でも指令を出すノードは上がらない**ので、そのまま先へ進んでよい。
機体のコードを更新する必要は無い (更新は失敗のリスクを増やすだけ)。

30 秒待つ。別の窓 (Ctrl-b c) で:

```bash
source ~/ros2-ws/install/setup.bash
ros2 control list_controllers
```
期待値: **6 個すべてが `active`**。

```
thruster_controller_lf/lb/rb/rf   active
attitude_controller               active
gate_controller                   active
```

1 つでも `inactive` / `unconfigured` があれば**止めて連絡**。

### 1-3. センサが出ているか

```bash
for t in /state/imu /state/thruster_state_all /state/high_power_circuit_info; do
  echo "--- $t"; timeout 8 ros2 topic hz -w 50 $t 2>&1 | tail -2
done
```
期待値: **どれも約 50 Hz**。`average rate: 49.9`〜`50.1` なら正常。

```bash
ros2 topic echo --once /state/high_power_circuit_info
```
期待値: `esc_*_state.voltage` が **4 つとも 22〜25 V**。
**20 V を下回っていたら充電されていない** — 止めて連絡。
`water_leaked` が `true` のものがあれば**直ちに止めて連絡**。

### 出ていても正常なもの (慌てないこと)

以下は**既知の未実装によるもので、故障ではない**。この日に直さない。

- ログに `Failed to write Can: Not implemented for ESC allowed command` などが
  **3 秒おきに延々と出る** — CAN 書き込みの一部が未実装なため。正常。
- `/state/high_power_circuit_info` の **トップレベルの `voltage` / `current` /
  `temperature` が 0.0** — メイン電源基板の読み出しが未実装。ESC 個別の電圧が本物。
- `/state/low_power_circuit_info` の **`headlights` と `indicator_led` が 1 (ERROR)** —
  ソフト側の既知の不具合で、ライトの故障ではない。
- IMU のログに `previous async trigger is still in progress` が数分に数回 — 正常。

---

## 2. 走行 bag の収録 (本命)

### なぜこれが要るか

いま **「指令 duty から実際に何ニュートン出ているか」が分かっていない**。
モデルの指数が 2.0 なのか 1.0 なのかで到達速度の予測が 2 倍以上変わる。

`/state/thruster_state_all` には **`rpm` が実測で入る**。推力は rpm の 2 乗に比例する
(これは物理として確か) ので、**`duty` と `rpm` の関係が分かれば指数が決まる**。
秤も治具も要らない。**必要なのは「duty を段階的に振って走らせた bag」だけ**。

過去の bag は disarm 状態 (スラスタを回していない) で録られていて使えなかった。
**今回は必ず arm して回すこと。**

### 2-1. 記録を先に開始する

**スタックより後でよいが、走らせる前に必ず開始する。** 新しい窓で:

```bash
cd ~/ros2-ws/src/sinsei_UMIUSI_autonomy/tools
./record_run.sh --bag-only --name 20260912-duty-sweep
```

20 秒後に「何を購読できたか」が出る。**`/state/thruster_state_all` が一覧に
無ければ録れていない** — Ctrl-C して 1-2 からやり直す。

### 2-2. duty を段階的に振る

**水中で行う。空中でプロペラを回さないこと。**

**機体は固定する (係留するか手で押さえる)。** 自走させてはいけない。理由は安全だけでなく
測定の質: プロペラの回転数は**流入速度で変わる**ので、機体が進むと同じ duty でも rpm が
変わってしまい、`duty -> rpm` の関係が汚染される。**止まった状態で測るのが正しい。**
固定できない場合は、少なくとも壁から十分離し、**前後に振られることを見込んだ場所**を取る
(下記のとおり正転と逆転の両方を回すので、機体は前後に押される)。

指令は `thruster_cmd.py` の `sweep` から出す。**この用途のために用意されている**
サブコマンドで、duty を段ごとに保持し、arm と終了時の disarm も自分で面倒を見る。

**1 基ずつ回す。** 4 基まとめてではなく個別に振ると、スラスタごとに当てはめができる
(1 基だけ固着している場合も切り分けられる)。別の窓で、`lf` → `lb` → `rb` → `rf` の順に:

```bash
cd ~/ros2-ws/src/sinsei_UMIUSI_autonomy/tools
./thruster_cmd.py sweep --ch lf --points 0.05 0.10 0.15 0.20 0.25 --dwell 12 --rest 3
```

`--dwell 12` は**各段を 12 秒保持**する指定。立ち上がりの数秒は錆や固着で rpm が
揃わないので、短いと使えない。既定の 5 秒では足りないので**必ず指定する**。

**指定した 5 点は正転と逆転の両方が回る** (`0.05, -0.05, 0.10, -0.10, ...` の 10 段)。
これは意図どおり — 逆転側のデータも同定に使う。**1 基あたり 10 段 × 15 秒 = 約 2.5 分**、
4 基で約 10 分。

開始前に実行内容が表示され Enter 待ちになる。**表示された duty の並びを確認してから
Enter を押す。** 想定と違っていたら Ctrl-C。

> プロンプトに「**秤の読みを各 dwell ごとに記録**」と出るが、**この日は秤を使わない**。
> このサブコマンドは元々ベンチ用に書かれているためで、無視してよい。
> こちらが要るのは bag に入る `rpm` だけ。

**`--allow-full` は付けないこと。** 付けないと duty 0.4 を超えられない安全装置が効く。
この日は 0.25 までしか使わないので、そもそも触る必要が無い。

**1 基終わるごとに次のコマンドを打つ** (`--ch lb`, `--ch rb`, `--ch rf`)。まとめて流さない。

各基の実行中に、別の窓で `rpm` が動いているか目で確認する:

```bash
ros2 topic echo --once /state/thruster_state_all | grep -A4 "^lf:"
```
`duty_cycle` が指令どおりで、**`rpm` が 0 でなく増えていれば成功**。

> `rpm` が 0 のまま duty だけ上がっている場合、**そのスラスタは回っていない**。
> 固着かもしれないので、一度 0 に戻して低 duty で数秒回してから再試行する。
> それでも 0 なら、そのスラスタは記録にメモして先に進む (無理に上げない)。

### 2-3. 止める

`sweep` は終了時に**ゼロ出力と disarm を自分で送ってから**抜ける。4 基ぶん終わったら
**記録の窓で Ctrl-C**。bag が閉じる。

途中で止めたいときも **`thruster_cmd.py` の窓で Ctrl-C** すればゼロ出力と disarm が
送られる。記録は後から止めてよい。

### 中止する条件 (迷ったら止める)

- `water_leaked` が `true` になった
- ESC 電圧が 20 V を下回った
- 異音・異臭・発熱
- 1 基だけ極端に挙動が違う
- **判断に迷った** — 止めて連絡するのが常に正しい。録り直しは安い

## 3. bag の回収

**録っただけで満足しないこと。** 8 月に 1 回、記録が機体の再起動で失われている。

機体の上で:

```bash
ls -la ~/runs/latest/
du -sh ~/runs/latest/
```

手元の PC から (機体の中からではない):

```bash
scp -r pi@umiusi2.local:runs/latest/ ./20260912-duty-sweep/
```

`~/runs/` は再起動しても消えないが、**回収するまで完了ではない**。
`/tmp` の下に何か置いた場合はそれも回収する (再起動で消える)。

## 4. 記録に残すこと

次の人が bag を見るときに要る:

- `git log --oneline -1` の出力 (0 でメモしたもの)
- **各段で実際に指令した duty** と保持時間 (既定から変えた場合)
- **機体を固定したかどうか** (係留 / 手で押さえた / 自走させた)。同定の前提が変わる
- 水温・水深のような環境条件 (分かる範囲で)
- 回らなかったスラスタがあればどれか
- 途中で止めた場合はその理由

---

## 回収した bag の見方 (担当者向け、現場では不要)

```bash
python3 tools/duty_rpm_fit.py <bag-dir>
```

`rpm = a * duty` の当てはめと決定係数を出す。線形に近ければ指数は 2.0 に近く、
飽和していれば 2.0 未満。前提と注意は `thrust_calibration.md`。

`esc_mode` が `Runnable` のサンプルが無いと「回帰に使えない」と報告して終わる —
**arm 忘れがこれで分かる。**

## 関連

- `experiment_guide.md` — 較正実験の全体像 (この 1 枚はその一部を抜き出したもの)
- `thrust_calibration.md` — なぜ duty -> rpm を測るのか、経路ごとの推力仮定の食い違い
- `known_issues.md` — 上の「出ていても正常なもの」の出どころ (B-8 / A-1)
- `robot_setup.md` — 機体に入れない / can0 が上がらないときの復旧
