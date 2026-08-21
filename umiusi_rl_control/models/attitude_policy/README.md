# attitude_policy — 姿勢保持だけのポリシー

`Umiusi_sim` の `models/att_v8`（`task: attitude`）を素形式に書き出したもの。
**前進成分を持たない**ので、狭いプールや空中でのドライ試験のように「姿勢だけ見たい」場面で使う。

| | `cruise_policy` (既定) | `attitude_policy` |
|---|---|---|
| タスク | `attitude_velocity` | `attitude` |
| 観測次元 | 25（`v_cmd` を含む） | **22**（`v_cmd` を含まない） |
| 速度指令 | 学習時 0.4 m/s。**0 にすると分布外**（`known_issues.md` A-9） | **無視される**（観測に無い） |

`rl_attitude_node` は読み込んだポリシーの入力次元で観測の組み立てを切り替えるので、
`model_path` を差し替えるだけで使える:

```bash
ros2 launch umiusi_rl_control rl_attitude.launch.py \
    model_path:=$(ros2 pkg prefix umiusi_rl_control)/share/umiusi_rl_control/models/attitude_policy
```

`vel_cmd` や `AttitudeTarget.velocity` を与えても**効きません**（ログで一度警告します）。

## 書き出しについて

`final.zip` は同梱していない（実機の numpy 1.26 では SB3 の zip が読めないため。`known_issues.md`
の該当項目を参照）。`export/` の中身だけで torch のみで推論できる。
**SB3 の出力と 200 サンプルで完全一致（最大差 0.000e+00）**を確認済み。

書き出し元は `Umiusi_sim` の `models/att_v8`（PPO / 600k steps / vecnormalize あり）。
