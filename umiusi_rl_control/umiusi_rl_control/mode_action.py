"""レンチモード action (``action_mode: "modes"``) を [servo x4, esc x4] に直す deploy 側の実装。

`av_mode13` 以降の方策は **[servo x4, esc x4] を出さない**。6 次元の「機体レンチのモード
**レート**」(REP-103、[fx, fy, fz, tx, ty, tz]) を出すので、sim と**同じ 3 段を同じ順で**
再現しないと指令が意味を持たない (A-11 と同型の sim2real 事故になる)。

なぜモード空間なのか (Umiusi_sim#3): 生の 8 次元 action には「何もしない」パターンが 2 つ
(鉛直・水平の零空間) 存在し、報酬整形では消しきれなかった (8/25 実機で鉛直パワーの 41%、
`w_null` を上げても ~22% で頭打ち)。**その 2 つを表現できない基底に action を張り替えた**のが
このパラメータ化で、零空間シェアは sim 実測 5.7% まで落ちている。

3 段の契約は **バンドルの ``export/meta.json`` の ``action_contract``** が正で、この実装は
そこから係数と符号表を読む (ハードコードしない — sim 側が変えたら読み込み時に落ちる)。

1. **積分**: ``m += a * mode_slew_per_s * dt`` を [-1, 1] にクリップ。``m`` はステップ間で
   保持し、**disarm でゼロに戻す** (`reset()`)。action は「モードのレート」なので、
   レート制限が方策の内側に入っている。
2. **ミキサ**: ユニット毎に ``h = Sh @ (fx, fy, tz) * f_max`` / ``v = Sv @ (fz, tx, ty) * f_max``、
   ``f_max = thrust_per_cmd * max_duty ** thrust_curve_exp``。
3. **折返し**: ``servo = atan2(v, h)`` を ±servo_range に折り返し (はみ出す側は esc 符号反転)、
   ``esc = sign * (min(|f|, f_max) / thrust_per_cmd) ** (1 / thrust_curve_exp)``。
   デッドバンド内のユニットは**前回のサーボ角を保持**し esc を 0 にする (atan2 が原点で
   定義できず、サーボがばたつくため)。

**平滑化の置き場所** — ここは 2 つの別物を取り違えやすい:

1. **バンバン制御を抑えるための平滑化は、モード座標に置く (上の 1 段目)。折返し後の
   servo/esc 座標に置き換えてはいけない。** 零空間の無い指令は per-unit の (h, v) 力空間の
   **線形部分空間**を成し、モード座標はその線形座標なので、モードを滑らかに動かせば途中も
   零空間ゼロのまま。折返し後の座標で内挿すると部分空間の**外**を通り、sim 実測で realized
   null が 19.7% (対 ~14%) に**増え**、サーボが固まった。
2. **プラントのレート制限は別。ノード側で従来どおり掛ける** (`rl_attitude_node._command` の
   `servo_slew_deg_per_s` 250 / `thrust_slew_per_s` 4.0)。sim は `simulator.py` の `step()` で
   `track`/`slew` を**毎サブステップ**適用してから推力を計算しており、方策はその鈍った系を
   前提に学習している。**特に ESC 側は実機に等価物が無い** — control の
   `max_duty_step_per_sec` は `/cmd/direct` 経路では素通りする (known_issues B-12) ので、
   ノードが掛けなければ誰も掛けない。外すと A-11 (レート制限が学習に入っていなかった) と
   同型の sim2real ずれになる。サーボも sim の 250 deg/s は実機 (データシート 300〜350) の
   保守側の推定値なので、ソフトで 250 に抑えるほうが学習時の応答に近い。
"""

from __future__ import annotations

import numpy as np

MODE_DIM = 6
ACT_DIM = 8
# meta.json から読む値。欠けていたら落とす — 既定値で埋めると sim と静かにずれる。
_REQUIRED = ("mode_names", "mode_signs", "mode_sign_columns", "mode_slew_per_s",
             "deadband_frac", "thrust_per_cmd", "thrust_curve_exp", "servo_range_deg")


class ModeAction:
    """モードレート action の状態 (積分器 + 前回サーボ角) と 3 段の変換をまとめて持つ。

    ノード側では **1 インスタンス = 1 ポリシー**。disarm のたびに `reset()` すること
    (積分器を残すと、次に武装した瞬間に前回の姿勢を保とうとする指令が出る)。
    """

    def __init__(self, contract: dict, positions):
        missing = [k for k in _REQUIRED if k not in contract]
        if missing:
            raise ValueError(
                f"action_contract に {missing} がありません。レンチモードのバンドルは "
                "meta.json の action_contract に 3 段ぶんの係数を含んでいる必要があります "
                "(sim 側の export が古い可能性)")
        names = list(contract["mode_names"])
        if len(names) != MODE_DIM:
            raise ValueError(f"mode_names は {MODE_DIM} 個必要です: {names}")
        cols = list(contract["mode_sign_columns"])
        if sorted(cols) != sorted(names):
            raise ValueError(
                f"mode_sign_columns {cols} が mode_names {names} と一致しません")
        signs = contract["mode_signs"]
        if set(signs) != set(positions):
            raise ValueError(
                f"mode_signs のユニット名 {sorted(signs)} がノードの配置 {list(positions)} と"
                "一致しません")
        # 長さも**ユニット名を添えて**確かめる。ここを素の np.array に任せると
        # 「inhomogeneous shape」という numpy の一般論に化けて、どのユニットが壊れているか
        # 分からなくなる (落ちること自体は変わらないが、実験の合間に読むログとして役に立たない)
        for p in positions:
            if len(signs[p]) != MODE_DIM:
                raise ValueError(
                    f"mode_signs['{p}'] は {MODE_DIM} 要素必要ですが {len(signs[p])} 個です "
                    f"({signs[p]})。列の並びは mode_sign_columns を参照")
        # 符号表は (h 3 列 | v 3 列) の並び。どのモード成分に掛かるかは mode_sign_columns で
        # 決まるので、順序を仮定せず名前で引く。
        s = np.array([[float(x) for x in signs[p]] for p in positions], dtype=float)
        if s.shape != (len(positions), MODE_DIM):
            raise ValueError(f"mode_signs の形が {s.shape} です ({len(positions)}, {MODE_DIM}) が必要")
        idx = [names.index(c) for c in cols]
        self._Sh, self._h_idx = s[:, 0:3], idx[0:3]
        self._Sv, self._v_idx = s[:, 3:6], idx[3:6]

        self.mode_slew_per_s = float(contract["mode_slew_per_s"])
        self.deadband_frac = float(contract["deadband_frac"])
        self.thrust_per_cmd = float(contract["thrust_per_cmd"])
        self.thrust_curve_exp = float(contract["thrust_curve_exp"])
        self.servo_range_deg = float(contract["servo_range_deg"])
        self.control_rate_hz = float(contract.get("control_rate_hz", 0.0))
        self._n = len(positions)
        self.reset()

    def reset(self) -> None:
        """積分器と前回サーボ角を初期状態に戻す。**disarm のたびに呼ぶ。**"""
        self._m = np.zeros(MODE_DIM)
        self._prev_servo = np.zeros(self._n)      # 正規化 (±1 = ±servo_range_deg)

    @property
    def modes(self) -> np.ndarray:
        """いま積分されているモードベクトル (診断用)。"""
        return self._m.copy()

    def step(self, raw, max_duty: float, dt: float) -> np.ndarray:
        """モードレート action -> [servo x4, esc x4] (各 [-1, 1])。

        raw:      方策の生出力 (6,)。**モードのレート**であって指令値ではない
        max_duty: いまの esc 上限。**方策が観測している値と同じものを渡すこと**
                  (モード 1.0 = 「その上限での全権限」という約束なので、食い違うと
                  方策の意図した力と実際の力がずれる)
        dt:       制御周期 [s]
        """
        a = np.clip(np.asarray(raw, dtype=float).reshape(MODE_DIM), -1.0, 1.0)
        # 1. 積分 (レート制限は方策の内側)
        self._m = np.clip(self._m + a * self.mode_slew_per_s * dt, -1.0, 1.0)
        m = self._m
        # 2. ミキサ
        f_max = self.thrust_per_cmd * float(max_duty) ** self.thrust_curve_exp
        h = self._Sh @ m[self._h_idx] * f_max      # 接線方向の力 [N]
        v = self._Sv @ m[self._v_idx] * f_max      # 鉛直方向の力 [N]
        # 3. 折返し
        phi = np.arctan2(v, h)                     # (-pi, pi]
        rear = np.abs(phi) > np.pi / 2.0           # 到達できない半平面 -> 折り返して esc 反転
        phi = np.where(rear, phi - np.sign(phi) * np.pi, phi)
        esc_sign = np.where(rear, -1.0, 1.0)
        mag = np.hypot(h, v)
        u = esc_sign * (np.minimum(mag, f_max) / self.thrust_per_cmd) ** (1.0 / self.thrust_curve_exp)
        servo = phi / np.radians(self.servo_range_deg)
        dead = mag < self.deadband_frac * f_max    # 原点近傍: 前回のサーボ角を保持
        servo = np.where(dead, self._prev_servo, servo)
        u = np.where(dead, 0.0, u)
        servo = np.clip(servo, -1.0, 1.0)
        self._prev_servo = servo
        return np.concatenate([servo, np.clip(u, -1.0, 1.0)])
