# ゲームパッドで操縦する — 姿勢制御を効かせたまま

Logicool のゲームパッド（コード内の確認済み機種は **F310**）で、**roll/pitch/yaw の安定化を
効かせたまま**手動操縦する手順と、ボタンの割り当て。

従来の FF 経路（`/cmd/target` → control の C++ feed-forward）とは**指令の意味が違う**ので、
同じスティックでも挙動が変わる。そこが本文の主題。

---

## まず動かす

```bash
# control は別に上げておく (umiusi_stack.sh start --control-only など)
ros2 launch umiusi_autonomy scenario.launch.py use_perception:=false \
    cmd_target_topic:=/cmd/target
ros2 param set /classical_attitude cmd_target_yaw_mode rate   # ← これが無いと 11 度しか回れない
ros2 service call /classical_attitude/arm std_srvs/srv/SetBool '{data: true}'
```

UI（ブラウザ）は `/user_input/target` に publish し、core の `manual_target_generator` が
それを `/cmd/target` へ**そのまま中継**する。姿勢制御器はそれを目標として受け、安定化と
4 基への配分をやって `/cmd/direct` に出す。

止めるのは `arm` サービスに `{data: false}`。**`~/estop` への `ros2 topic pub` は QoS が
合わず届かない**（`docs/scenario_run.md`）。

---

## ボタンの割り当て

`sinsei_UMIUSI_ui/src/utils/gamepadMapping.ts` と `src/services/gamepadPublisher.ts` が正。

| 操作 | UI が入れるフィールド | 値 | 姿勢制御を効かせたときの意味 |
|---|---|---|---|
| 左スティック 上下 | `velocity.x` | `-axes[1]`、[−1, 1] | **前後（surge）**。1.0 が「その cap で到達できる速度」 |
| 左スティック 左右 | `orientation.z` | `-0.2 * axes[0]` | **旋回**。`rate` モードで**倒している間ずっと回る** |
| 右スティック 左右 | `orientation.x` | `0.3 * axes[2]` | **ロール目標**。最大 ±17°。放すと水平に戻る |
| 右スティック 上下 | `orientation.y` | `-0.3 * axes[3]` | **ピッチ目標**。最大 ±17°。放すと水平に戻る |
| 十字キー 左 / 右 | `velocity.y` | `±0.5` | **横移動（sway）**。押している間だけ |
| L2 / R2 | `velocity.z` | `±0.3 * トリガ量` | **上下（heave）**。L2 が上、R2 が下 |
| A/B/X/Y, L1/R1, 十字 上下 | — | — | **未割り当て**（`gamepadMapping.ts` の FIXME） |

スティックには 0.1 のデッドゾーンが入っている（`deadzone()`）。送信は 30 Hz。

### 従来の FF 経路と何が違うか

| | FF 経路（`logic/attitude/feed_forward.hpp`） | 姿勢制御経路（この文書） |
|---|---|---|
| `orientation` の意味 | **無次元のトルク指令**。配分行列へ直入れ | **絶対角 [rad]**（`rl.hpp` と同じ規約） |
| 右スティックを倒す | その向きに回し続ける | **その角度に傾いて止まる**。放すと水平へ戻る |
| 左スティック左右 | 旋回し続ける | `rate` モードなら同じ。`absolute` だと **±11° で止まる** |
| 手を放したとき | 出力が 0 になるだけ（姿勢は成り行き） | **水平・現在方位を保ち続ける** |
| 波や推力の非対称 | 誰も戻さない | 姿勢制御器が戻す |

> **`cmd_target_yaw_mode` の既定は `absolute`**（control の `rl.hpp` に合わせた規約どおり）。
> ゲームパッドで操縦するなら **`rate` に変えること**。`absolute` のままだと、UI が出す
> ±0.2 rad がそのまま目標角になるので **±11° しか回れない**。

---

## Logicool のパッドで最初に確認すること

### X / D スイッチ

F310 / F710 には**背面に `X` / `D` の切り替えスイッチ**がある。ここが**最大の落とし穴**:

- **`X`（XInput）** … ブラウザが "standard gamepad" として認識し、`gamepadMapping.ts` の
  添字（axes 0–3 / buttons 0–15、十字キーは 12–15）がそのまま当たる。**こちらにする。**
- **`D`（DirectInput）** … 添字の並びが変わり、十字キーがボタンではなく軸になる。
  **エラーは出ず、操作が静かにズレるだけ。** 前進のつもりで旋回する、といった形で出る。

切り替えたら**パッドを挿し直し、ブラウザのタブも再読み込みする**（Gamepad API は接続時の
マッピングを掴んだまま離さない）。

### 割り当てが合っているかの確かめかた

**機体を動かす前に、水から出した状態で**指令だけを見る。

```bash
ros2 topic echo /user_input/target      # UI が出しているもの
ros2 topic echo /cmd/target             # core が中継したもの (同じはず)
```

順に倒して、下の表と合っているかを見る:

| 動かすもの | `orientation` / `velocity` のどこが動くべきか |
|---|---|
| 左スティックを**上** | `velocity.x` が **+** |
| 左スティックを**左** | `orientation.z` が **+**（`-0.2 * axes[0]`、左倒しで axes[0] は負） |
| 右スティックを**右** | `orientation.x` が **+** |
| 右スティックを**上** | `orientation.y` が **+** |
| L2 | `velocity.z` が **+** |

**1 つでも違う軸が動いたら `D` モードを疑う。** それでも合わなければ機種が違うので
`gamepadMapping.ts` の添字を直す（`// TODO: コントローラーの種類ごとにマッピングを変える`）。

### 機体をつないでからの確かめかた

**係留して、duty 上限を下げてから。**

```bash
ros2 param set /classical_attitude max_duty 0.15
ros2 service call /classical_attitude/arm std_srvs/srv/SetBool '{data: true}'
```

左スティックを軽く上へ → **前に進むこと**。後退したら、パッドではなく**推力の符号**を
疑う（`tools/thrust_sign_check.py`。`docs/scenario_run.md`）。

---

## 効きを調整する

```bash
ros2 param set /classical_attitude cmd_target_yaw_rate_scale 1.0   # 旋回をゆっくり (既定 1.5 rad/s)
ros2 param set /classical_attitude cmd_target_vel_scale 0.1        # 前後をゆっくり [m/s]
ros2 param set /classical_attitude max_duty 0.4                    # 出力の上限
ros2 param set /classical_attitude k_v_vert 1.2                    # 上下の効きを上げる
```

| パラメータ | 既定 | 効く場面 |
|---|---|---|
| `cmd_target_yaw_mode` | `absolute` | **ゲームパッドなら `rate`** |
| `cmd_target_yaw_rate_scale` | 1.5 rad/s | 旋回が速すぎる / 遅すぎる |
| `cmd_target_yaw_lead_max` | 1.05 rad (60°) | 目標方位が実測から先行してよい上限。**下げると素直になるが旋回が遅くなる**。上げすぎると機体が追随できないまま目標が逃げる |
| `cmd_target_vel_scale` | 負 = 到達速度（cap 0.25 で約 0.21 m/s） | スティック 1.0 を何 m/s に対応させるか |
| `k_v_vert` | 0（前進項のみ） | 上下の追従。**鉛直の速度推定は指令からの推測**なので、上げるのは効きを実機で見てから |

---

## 既知の穴

- **未割り当てのボタンが多い。** A/B/X/Y・L1/R1・十字の上下は何にも繋がっていない
  （`gamepadMapping.ts` の FIXME）。e-stop をボタンに割り当てるなら UI 側の変更が要る。
- **UI は物理単位を知らない。** 送っているのは正規化したスティック値で、m/s でも rad でも
  ない。橋渡しは受け側（`cmd_target_vel_scale`）でやっている。UI 側を物理単位にすると
  **FF 経路の挙動も変わる**ので、変えるなら両方まとめて。
- **`velocity.y`（sway）は十字キーで ±0.5 の 2 値**。微調整できない。
- **UI とスタックが同時に `/cmd/target` を触れる。** core の `auto_target_generator` も
  同じトピックに出すので、自律と手動を同時に上げると取り合う。

## 関連

| ファイル | 中身 |
|---|---|
| `docs/scenario_run.md` | シナリオ全体の起動と、`/cmd/target` の規約 |
| `sinsei_UMIUSI_ui/src/utils/gamepadMapping.ts` | ボタン添字の正 |
| `sinsei_UMIUSI_ui/src/services/gamepadPublisher.ts` | どの軸をどのフィールドに入れるか |
| `sinsei_UMIUSI_core/.../manual_target_generator.py` | `/user_input/target` → `/cmd/target` の中継 |
