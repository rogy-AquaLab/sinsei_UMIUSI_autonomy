"""深度モード切替スーパーバイザ — 水圧の外側ループで 2 つの RL ポリシーを使い分ける。

規約:
  * depth: 正が深い [m]。depth_err = target - depth が正なら「もっと潜る」
  * 降下指令: v_cmd = [0, 0, -vz] (下 = -z。frame は known_issues A-13)

禁止事項 (sim 実測。テストでは捕まらない):
  * 水平ポリシーに鉛直指令を入れない — 分布外で姿勢が崩壊する
  * 浮上に指令を足さない — 3-D ポリシーは降下専用。浮上は弱正浮力に任せる
  * 前進したまま降下バーストに入らない — 転覆する。必ずブレーキを挟む
  * max_duty を上げて降下させようとしない — 0.2 で降下できない原因は上限ではなく
    配分 (零空間)。上限だけ上げると転覆余裕を削る (known_issues A-17)

既定値と sim での検証結果は issue #15 のコメント (tools/mode_switch_eval.py)。
"""
from __future__ import annotations

import numpy as np

# スーパーバイザの状態。HORIZ 以外は「深度補正中」で、水平巡航指令は適用されない。
HORIZ = "horiz"     # 水平ポリシー。要求どおりの水平巡航 + 姿勢保持
BRAKE = "brake"     # 降下バーストの前置き: 水平ポリシーでホールドし前進慣性を殺す
VERT = "vert"       # 3-D ポリシーに純下指令 (降下バースト)
ASCEND = "ascend"   # 水平ポリシーでホールドし浮力で浮上 (受動)


class DepthSupervisor:
    """深度誤差の閾値で水平/鉛直モードを切り替える状態機械 (ヒステリシス + watchdog 付き)。

    毎 tick `update(now, depth)` を呼ぶ。返り値は (state, v_cmd_override):
      * state が HORIZ のとき v_cmd_override は None — 要求どおりの水平指令を使ってよい
      * それ以外は補正中 — v_cmd_override (body frame, np.ndarray(3)) をそのまま
        ポリシーへ入れる (BRAKE/ASCEND は零ベクトル、VERT は純下)
    どのポリシーで推論すべきかは state で決まる (VERT だけ 3-D ポリシー)。
    """

    def __init__(self, d_enter=0.25, d_exit=0.15, d_exit_descend=0.20,
                 k_depth=0.7, v_vert=0.2, rate_gate=0.08, t_brake=1.0):
        self.d_enter = float(d_enter)            # 補正に入る深度誤差 [m]
        self.d_exit = float(d_exit)              # 浮上をやめる誤差 [m] (ヒステリシス)
        self.d_exit_descend = float(d_exit_descend)  # 降下をやめる誤差 [m] (慣性+浮力が残りを処理)
        self.k_depth = float(k_depth)            # 降下指令 vz = clip(k*err, 0, v_vert) [1/s]
        self.v_vert = float(v_vert)              # 降下指令の上限 [m/s]
        self.rate_gate = float(rate_gate)        # 降下 exit に要求する |深度レート| 上限 [m/s]
        self.t_brake = float(t_brake)            # ブレーキ時間 [s]
        self.target_depth = 0.0                  # 目標深度 [m, 正=深い]
        self.state = HORIZ
        self.switches = 0                        # 補正に入った回数 (HORIZ からの遷移)
        self.retries = 0                         # watchdog によるバーストやり直し回数
        self._brake_until = 0.0
        self._vert_since = 0.0
        self._hist: list[tuple[float, float]] = []   # (t, depth) — レート推定用リング

    def depth_rate(self):
        """直近 ~0.5 s の平均深度レート [m/s, 正=潜行中]。センサノイズに強い差分。"""
        if len(self._hist) < 2:
            return 0.0
        (t0, d0), (t1, d1) = self._hist[0], self._hist[-1]
        return (d1 - d0) / (t1 - t0) if t1 > t0 else 0.0

    def update(self, now, depth):
        """-> (state, v_cmd_override | None)。now [s] は単調増加であればよい。"""
        self._hist.append((float(now), float(depth)))
        while self._hist and now - self._hist[0][0] > 0.5:
            self._hist.pop(0)
        err = self.target_depth - depth          # 正 = もっと潜る
        rate = self.depth_rate()

        if self.state == HORIZ:
            if err > self.d_enter:               # 浅すぎる -> 降下バースト (ブレーキから)
                self.state = BRAKE
                self._brake_until = now + self.t_brake
                self.switches += 1
            elif err < -self.d_enter:            # 深すぎる -> 受動浮上
                self.state = ASCEND
                self.switches += 1
        elif self.state == BRAKE:
            if now >= self._brake_until:
                self.state = VERT
                self._vert_since = now
        elif self.state == VERT:
            if err < self.d_exit_descend and abs(rate) < self.rate_gate:
                self.state = HORIZ
            elif ((now - self._vert_since > 2.5 and rate < -0.03)
                  or (now - self._vert_since > 4.0 and rate < 0.02)):
                # 注: sim 版 (mode_switch_eval.py) と符号が逆に見えるのは正しい —
                # あちらは y-up (rate 正=上昇)、こちらは深度 (rate 正=潜行)
                # watchdog: バースト不発 (降下せず浮上/停滞)。2.5 s の猶予は正常な
                # 「浮上ドリフトを殺す」過渡をカバーする (健全なバーストも最初 ~2 s は浮く)
                self.state = BRAKE
                self._brake_until = now + self.t_brake
                self.retries += 1
        elif self.state == ASCEND:
            if err > -self.d_exit:
                self.state = HORIZ

        if self.state == VERT:
            vz_down = float(np.clip(self.k_depth * err, 0.0, self.v_vert))
            return self.state, np.array([0.0, 0.0, -vz_down])   # REP-103: 下 = -z
        if self.state in (BRAKE, ASCEND):
            return self.state, np.zeros(3)                      # ホールド (巡航一時停止)
        return self.state, None                                 # 水平モード: 要求どおり
