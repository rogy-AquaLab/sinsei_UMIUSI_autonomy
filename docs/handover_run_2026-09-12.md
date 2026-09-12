# 実機を回す手順 — 2026-09-12

**このリポジトリを知らない人が、この 1 枚だけで回せるように書いてある。**
上から順にやれば終わる。コマンドはコピペで動く。**迷ったら止めてよい** — 録り直しは安い。

- **やること**: スラスタの duty を段階的に振って、回転数を記録する
- **やらないこと**: 古典制御 / カメラの実験 / パラメータの調整 / 機体のコード更新
- **かかる時間**: 約 40 分（点検 10 分 + 記録 20 分 + 回収 5 分）

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

**止め方**: `thruster_cmd.py` の窓で `Ctrl-C`。ゼロ出力と disarm が自動で送られる。
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
> 指令を出すノードが上がると、手順 9 のツールと衝突する。

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

## 7. 機体を固定する

**係留するか、手で押さえる。自走させてはいけない。**

安全のためだけではない。**プロペラの回転数は流入速度で変わる**ので、機体が進むと同じ duty
でも回転数が変わってしまい、測ったデータが使えなくなる。**止まった状態で測るのが正しい。**

固定できない場合は壁から十分離す。**正転と逆転の両方を回すので機体は前後に押される。**

## 8. 記録を開始する（新しい窓）

**駆動より先に開始する。**

```bash
cd ~/ros2-ws/src/sinsei_UMIUSI_autonomy/tools
./record_run.sh --bag-only --name 20260912-duty-sweep
```

20 秒後に「何を購読できたか」が出る。

- **期待値**: 一覧に **`/state/thruster_state_all` がある**
- **ダメなとき**: 無ければ録れていない。`Ctrl-C` して手順 5 からやり直す

## 9. duty を振る（新しい窓）

**水中で行う。空中でプロペラを回さないこと。**

**1 基ずつ、4 回に分けて実行する。** `--ch` を `lf` → `lb` → `rb` → `rf` と変えて叩く:

```bash
cd ~/ros2-ws/src/sinsei_UMIUSI_autonomy/tools
./thruster_cmd.py sweep --ch lf --points 0.05 0.10 0.15 0.20 0.25 --dwell 12 --rest 3
```

実行内容が表示されて Enter 待ちになる。**表示された duty の並びを確認してから Enter。**
違っていたら `Ctrl-C`。

**1 基あたり約 2.5 分**（指定した 5 点が正転と逆転の両方回るので 10 段 × 15 秒）。
**1 基終わってから次を叩く。** まとめて流さない。

> - プロンプトに「**秤の読みを各 dwell ごとに記録**」と出るが、**この日は秤を使わない。**
>   このツールは元々ベンチ用。無視してよい。
> - **`--allow-full` は付けない。** 付けないと duty 0.4 を超えられない安全装置が効く。
> - **`--dwell 12` を省略しない。** 既定の 5 秒では立ち上がりの影響が抜けず使えない。

実行中に、別の窓で回転数を見る（`lf` 以外を回しているときは `^lf:` をその位置に変える）:

```bash
ros2 topic echo --once /state/thruster_state_all | grep -A4 "^lf:"
```

- **期待値**: `duty_cycle` が指令どおりで、**`rpm` が 0 でない**
- **`rpm` が 0 のまま duty だけ上がるとき**: そのスラスタは回っていない。一度 0 に戻して
  低い duty で数秒回してから再試行する。それでも 0 なら**そのスラスタをメモして先へ進む**
  （無理に duty を上げない）

## 10. 止める

4 基ぶん終わったら:

1. `thruster_cmd.py` は終了時に**ゼロ出力と disarm を自分で送る**ので、終わるのを待つ
2. **記録の窓で `Ctrl-C`**。bag が閉じる

## 11. 回収する

**録っただけでは終わっていない。** 8 月に 1 回、記録が機体の再起動で失われている。

機体の上で中身を確認:

```bash
ls -la ~/runs/latest/
du -sh ~/runs/latest/
```

**手元の PC から**（機体の中からではない）:

```bash
scp -r pi@umiusi2.local:runs/latest/ ./20260912-duty-sweep/
```

## 12. 記録に残す

次に見る人が必要とするもの:

- [ ] 手順 3 でメモした `git log` の行
- [ ] **機体を固定したか**（係留 / 手で押さえた / 自走させた）
- [ ] 既定から変えた duty や保持時間があれば、その値
- [ ] 回らなかったスラスタがあればどれか
- [ ] 途中で止めた場合はその理由
- [ ] 水温・水深など分かる範囲の環境条件

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
