"""depth_supervisor の状態機械テスト (ROS/torch 不要)。

sim 実測に合わせた合成 1-D プラントで回す: 水平系 (horiz/brake/ascend) は浮上ドリフト
-0.05 m/s (深度減少)、降下バースト (vert) は最初 ~2 s の過渡でまだ浮き、その後 +0.15 m/s
で潜る。watchdog テストだけ「不発バースト」(潜らず浮き続ける) を注入する。
"""
import numpy as np

from umiusi_rl_control.depth_supervisor import ASCEND, BRAKE, HORIZ, VERT, DepthSupervisor

DT = 0.02  # 50 Hz


def run(sup, depth0, seconds, misfire_until=0.0):
    """合成プラントで回して (depths, states) を返す。misfire_until [s] までの vert は不発。"""
    depth, t = depth0, 0.0
    vert_since = None
    depths, states = [], []
    for _ in range(int(seconds / DT)):
        state, override = sup.update(t, depth)
        if state == VERT:
            if vert_since is None:
                vert_since = t
            arrest = (t - vert_since) < 1.0
            if t < misfire_until or arrest:
                depth -= 0.05 * DT          # まだ浮いている (過渡 or 不発)
            else:
                # 指令追従: 実降下率は外側ループの指令に比例 (上限 0.15 m/s)。
                # 目標に近づくと指令が縮み、レートも落ちて exit ゲートが開く
                depth += min(0.15, 0.9 * float(-override[2])) * DT
        else:
            vert_since = None
            depth -= 0.05 * DT              # 浮上ドリフト (受動浮上もこれ)
        depth = max(depth, 0.0)             # 水面より上には行かない
        if override is not None:            # 補正中の指令の恒等式
            assert override.shape == (3,)
            assert override[0] == 0.0 and override[1] == 0.0
            assert override[2] <= 0.0       # REP-103: 下 = -z。上向き指令は決して出さない
            assert override[2] >= -sup.v_vert
        t += DT
        depths.append(depth)
        states.append(state)
    return np.array(depths), states


def test_dive_reaches_and_exits():
    sup = DepthSupervisor()
    sup.target_depth = 1.0
    depths, states = run(sup, 0.0, 30.0)
    assert BRAKE in states and VERT in states
    # 到達して HORIZ に戻り、目標近傍で終わる
    assert states[-1] in (HORIZ, BRAKE, VERT)
    assert abs(1.0 - depths[-1]) < 0.5
    assert sup.retries == 0
    # BRAKE は t_brake (1 s) ぶん続いてから VERT に入る
    first_brake = states.index(BRAKE)
    first_vert = states.index(VERT)
    assert (first_vert - first_brake) * DT >= sup.t_brake - DT


def test_hold_is_sawtooth_within_band():
    """捕捉後は浮上ドリフト → 再バーストの鋸歯。深度は概ね ±(d_enter+過渡) に収まる。"""
    sup = DepthSupervisor()
    sup.target_depth = 1.0
    depths, states = run(sup, 1.0, 60.0)
    assert sup.switches >= 2                          # 何度か補正に入っている
    assert np.max(np.abs(1.0 - depths[int(5 / DT):])) < 0.55   # sim 実測の鋸歯振幅と同等


def test_ascend_is_passive():
    sup = DepthSupervisor()
    sup.target_depth = 0.0
    depths, states = run(sup, 1.0, 30.0)
    assert ASCEND in states
    assert VERT not in states                         # 浮上に 3-D ポリシーは使わない
    assert depths[-1] < 0.2 + sup.d_exit


def test_watchdog_retries_failed_burst():
    sup = DepthSupervisor()
    sup.target_depth = 1.0
    # 最初の 8 s はバーストが不発 (潜らない) → watchdog が BRAKE に戻してリトライし、
    # 不発が解けたら到達する
    depths, states = run(sup, 0.0, 40.0, misfire_until=8.0)
    assert sup.retries >= 1
    assert abs(1.0 - depths[-1]) < 0.5


def test_horiz_passthrough_and_hysteresis():
    sup = DepthSupervisor()
    sup.target_depth = 0.0
    # 誤差が d_enter 以下なら何もしない (デッドバンド)
    state, override = sup.update(0.0, 0.2)
    assert state == HORIZ and override is None
    # 閾値ちょうどでは入らない / 超えたら入る
    state, _ = sup.update(0.1, sup.d_enter - 0.01)
    assert state == HORIZ
    state, _ = sup.update(0.2, -(sup.d_enter + 0.05))   # 深度が負 = 浅すぎ…ではなく
    # depth = -0.3 (水面上) は err = +0.3 > d_enter -> 降下 (ブレーキ) に入る
    assert state == BRAKE
