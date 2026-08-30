"""レンチモード action (action_mode: "modes") を [servo x4, esc x4] に直す。

av_mode13 以降の方策は 6 次元の「機体レンチのモードレート」を出す。
積分 -> ミキサ -> 折返しの 3 段を、sim と同じ順で通す。

呼び出し側の義務 (このモジュールのテストでは捕まらない):
  * 係数も符号表も export/meta.json の action_contract から読む。ハードコードすると
    sim 側の変更と静かにずれる
  * disarm のたびに reset() する。積分器を残すと再武装の瞬間に前回の力が出る
  * max_duty は方策が観測しているのと同じ値を渡す (モード 1.0 = その上限での全権限)
  * 平滑化を折返し後の servo/esc 座標に置き換えない — 部分空間の外を通り零空間が増える

3 段の変換そのものは test_mode_action.py が固定している。
"""

from __future__ import annotations

import numpy as np

MODE_DIM = 6
ACT_DIM = 8
# meta.json から読む値。欠けていたら落とす — 既定値で埋めると sim と静かにずれる。
_REQUIRED = ("mode_names", "mode_signs", "mode_sign_columns", "mode_slew_per_s",
             "deadband_frac", "thrust_per_cmd", "thrust_curve_exp", "servo_range_deg")


class ModeAction:
    """モードレート action の状態 (積分器 + 前回サーボ角) と 3 段の変換。1 インスタンス = 1 ポリシー。"""

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
        # np.array に任せると numpy の "inhomogeneous shape" に化けて、どのユニットが
        # 壊れているか分からなくなる
        for p in positions:
            if len(signs[p]) != MODE_DIM:
                raise ValueError(
                    f"mode_signs['{p}'] は {MODE_DIM} 要素必要ですが {len(signs[p])} 個です "
                    f"({signs[p]})。列の並びは mode_sign_columns を参照")
        # 符号表は (h 3 列 | v 3 列)。どのモード成分に掛かるかは mode_sign_columns が決めるので、
        # mode_names の順序を仮定せず名前で引く
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
        """disarm のたびに呼ぶ。積分器と前回サーボ角を初期状態に戻す。"""
        self._m = np.zeros(MODE_DIM)
        self._prev_servo = np.zeros(self._n)      # 正規化 (±1 = ±servo_range_deg)

    @property
    def modes(self) -> np.ndarray:
        """いま積分されているモードベクトル (診断用)。"""
        return self._m.copy()

    def step(self, raw, max_duty: float, dt: float) -> np.ndarray:
        """モードレート action -> [servo x4, esc x4] (各 [-1, 1])。

        raw はモードのレートであって指令値ではない。max_duty は方策が観測している値と
        同じものを渡すこと (冒頭の docstring 参照)。
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
