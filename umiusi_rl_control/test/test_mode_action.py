"""レンチモード action (`action_mode: "modes"`) の単体テスト。

**この方策の存在理由は「零空間を表現できないこと」**なので、そこを直接見る。8/25 の実機 run は
鉛直パワーの 41% が零空間 (どのユニットも打ち消し合って推進に寄与しないパターン) に流れており、
報酬整形では ~22% までしか落ちなかった。モード基底はそのパターンを**構造的に持たない**。

期待値は **契約 (meta.json の action_contract) の式から独立に**組み立てる。実装の定数を
参照すると、定数を間違える mutation でテストの期待値も一緒に動いて検出できない。
"""
import numpy as np
import pytest

from umiusi_rl_control.mode_action import MODE_DIM, ModeAction

POSITIONS = ("lf", "lb", "rb", "rf")

# **零空間ベクトル。** ユニット順 (lf, lb, rb, rf) で (+,-,+,-) — 水平・鉛直とも同じ形。
# 幾何から出るもので、実装から持ってこない: 符号表の 3 列 (fz, tx, ty) と (fx, fy, tz) は
# それぞれ Walsh ベクトルで、この 4 本目だけがどの列とも直交する = どの指令でも作れない。
NULL_VEC = np.array([+1.0, -1.0, +1.0, -1.0])

# 契約の写し (sim の export と同じ値)。テストが実装の meta 読み出しに依存しないよう直に書く。
CONTRACT = {
    "mode_names": ["fx", "fy", "fz", "tx", "ty", "tz"],
    "mode_sign_columns": ["fx", "fy", "tz", "fz", "tx", "ty"],
    "mode_signs": {
        "lf": [1, -1, -1, 1, 1, -1],
        "lb": [1, 1, -1, 1, 1, 1],
        "rb": [-1, 1, -1, 1, -1, 1],
        "rf": [-1, -1, -1, 1, -1, -1],
    },
    "mode_slew_per_s": 2.0,
    "deadband_frac": 0.02,
    "thrust_per_cmd": 30.0,
    "thrust_curve_exp": 2.0,
    "servo_range_deg": 90.0,
    "control_rate_hz": 50.0,
}
DT = 1.0 / 50.0
MAX_DUTY = 0.25
F_MAX = 30.0 * MAX_DUTY ** 2.0


def _forces(action, max_duty=MAX_DUTY):
    """出力 [servo x4, esc x4] からユニット毎の (h, v) 力 [N] を復元する。

    折返しの逆: esc の符号が向きを、|esc| が大きさを持つ。
        |f| = thrust_per_cmd * |esc| ** thrust_curve_exp,  向き = servo 角 (esc 符号込み)
    """
    servo, esc = np.asarray(action[:4]), np.asarray(action[4:])
    phi = servo * np.radians(90.0)
    f = np.sign(esc) * 30.0 * np.abs(esc) ** 2.0
    return f * np.cos(phi), f * np.sin(phi)


def _drive(ma, raw, steps, max_duty=MAX_DUTY):
    """同じレートを `steps` 回入れて、最後の action を返す。"""
    out = None
    for _ in range(steps):
        out = ma.step(raw, max_duty, DT)
    return out


def _rate(**modes):
    """名前でモードレートを作る (順序の取り違えをテスト側に持ち込まない)。"""
    a = np.zeros(MODE_DIM)
    for k, v in modes.items():
        a[CONTRACT["mode_names"].index(k)] = v
    return a


# --- 中核: 零空間が作れないこと -----------------------------------------------------------

@pytest.mark.parametrize("seed", range(12))
def test_どのモード指令でも零空間成分は出ない(seed):
    """線形域では、実現した (h, v) の (+,-,+,-) 成分は厳密にゼロ。

    線形域の外は 2 つある。どちらも「モード基底が零空間を持たない」こととは別の話なので、
    ここでは除外して数える:
      * per-unit クリップ (|f| > f_max) — 飽和は非線形。方策は prev_action で見て学習している
      * デッドバンド (|f| < 2% f_max) — esc を切って角度を保持する。原点近傍の角度不定を
        避けるための意図的な処理で、そのぶん微小な零空間が出る
    """
    rng = np.random.default_rng(seed)
    ma = ModeAction(CONTRACT, POSITIONS)
    checked = 0
    for _ in range(200):
        a = ma.step(rng.uniform(-0.4, 0.4, MODE_DIM), MAX_DUTY, DT)
        h, v = _forces(a)
        mag = np.hypot(h, v)
        if np.any(mag >= F_MAX * 0.999) or np.any(mag <= F_MAX * 0.02 * 1.001):
            continue                      # 非線形域 (飽和 / デッドバンド)
        checked += 1
        assert abs(float(h @ NULL_VEC)) < 1e-9, f"水平に零空間成分: {h}"
        assert abs(float(v @ NULL_VEC)) < 1e-9, f"鉛直に零空間成分: {v}"
    # 全部除外されて素通りするのを防ぐ (テストが空回りしていないことの確認)
    assert checked >= 20, f"線形域のサンプルが {checked} 件しかない — テストが効いていない"


def test_零空間パターンはそもそも指令できない():
    """(+,-,+,-) の力を作ろうとしても、6 つのモードのどの組み合わせでも到達しない。"""
    rng = np.random.default_rng(0)
    best = np.inf
    for _ in range(2000):
        ma = ModeAction(CONTRACT, POSITIONS)
        a = _drive(ma, rng.uniform(-1, 1, MODE_DIM), 30)
        _, v = _forces(a)
        # 零空間方向の成分を最大化したい: |v·n| / |v| が 1 に近づけば「作れた」ことになる
        n = float(np.linalg.norm(v))
        if n > 1e-9:
            best = min(best, 1.0 - abs(float(v @ NULL_VEC)) / (2.0 * n))
    assert best > 0.49, f"零空間方向に寄せられてしまった (残差 {best:.3f})"


# --- 3 段の契約 ---------------------------------------------------------------------------

def test_純粋な上向き指令は全ユニット上向き全開になる():
    """fz を 1.0 まで積むと h=0 / v=+f_max、つまり servo=+90 度・esc=max_duty。"""
    ma = ModeAction(CONTRACT, POSITIONS)
    # slew 2.0/s、dt 0.02 -> 1 step 0.04。25 step で m=1.0 に飽和する
    a = _drive(ma, _rate(fz=1.0), 25)
    assert np.allclose(a[:4], 1.0), f"servo が +90 度になっていない: {a[:4]}"
    assert np.allclose(a[4:], MAX_DUTY), f"esc が上限になっていない: {a[4:]}"
    h, v = _forces(a)
    assert np.allclose(h, 0.0, atol=1e-9)
    assert np.allclose(v, F_MAX)


def test_積分はレート制限そのもの():
    """1 step で動けるのは mode_slew_per_s * dt まで (指令値ではなくレートを受け取る)。"""
    ma = ModeAction(CONTRACT, POSITIONS)
    ma.step(_rate(fz=1.0), MAX_DUTY, DT)
    assert ma.modes[CONTRACT["mode_names"].index("fz")] == pytest.approx(2.0 * DT)


def test_dutyはmax_dutyを超えない():
    """飽和させ続けても esc は上限内。ミキサの構成上そうなる (別クリップに頼らない)。"""
    rng = np.random.default_rng(1)
    for max_duty in (0.2, 0.25, 0.4):
        ma = ModeAction(CONTRACT, POSITIONS)
        for _ in range(200):
            a = ma.step(rng.choice([-1.0, 1.0], MODE_DIM), max_duty, DT)
            assert np.all(np.abs(a[4:]) <= max_duty + 1e-12), f"cap {max_duty}: {a[4:]}"


def test_デッドバンドでは前回のサーボ角を保ちescを切る():
    """原点近傍は atan2 が定義できない。角度を保持して esc だけ 0 にする (ばたつき防止)。"""
    ma = ModeAction(CONTRACT, POSITIONS)
    held = _drive(ma, _rate(fz=1.0), 25)[:4]        # +90 度に振ってから
    back = _drive(ma, _rate(fz=-1.0), 25)           # 原点へ戻す (m=0)
    assert np.allclose(back[4:], 0.0), f"デッドバンド内で esc が出ている: {back[4:]}"
    assert np.allclose(back[:4], held), f"サーボ角が保持されていない: {back[:4]} vs {held}"


def test_resetで積分器と保持サーボ角が初期化される():
    ma = ModeAction(CONTRACT, POSITIONS)
    _drive(ma, _rate(fz=1.0, tx=0.5), 25)
    ma.reset()
    assert np.all(ma.modes == 0.0)
    a = ma.step(np.zeros(MODE_DIM), MAX_DUTY, DT)
    assert np.all(a == 0.0), f"reset 後にゼロレートで指令が出た: {a}"


def test_max_dutyを上げると同じモードでより大きな力になる():
    """モード 1.0 = 「その上限での全権限」。上限を上げたら実際に強くなる (18 次元化の目的)。"""
    forces = []
    for cap in (0.2, 0.4):
        ma = ModeAction(CONTRACT, POSITIONS)
        a = _drive(ma, _rate(fz=1.0), 25, max_duty=cap)
        assert np.allclose(a[4:], cap), f"cap {cap} で esc が上限になっていない: {a[4:]}"
        forces.append(float((30.0 * np.abs(a[4:]) ** 2.0).max()))
    # f_max = 30 * cap^2 なので、上限を 2 倍にすれば力は 4 倍
    assert forces[1] == pytest.approx(4.0 * forces[0]), \
        f"上限を倍にしたのに力が 4 倍になっていない: {forces}"


# --- 契約の検証 (壊れたバンドルを黙って動かさない) --------------------------------------------

@pytest.mark.parametrize("missing", ["mode_signs", "mode_slew_per_s", "thrust_per_cmd",
                                     "servo_range_deg", "mode_sign_columns"])
def test_契約が欠けていたら落とす(missing):
    c = {k: v for k, v in CONTRACT.items() if k != missing}
    with pytest.raises(ValueError, match="action_contract"):
        ModeAction(c, POSITIONS)


def test_ユニット名が食い違ったら落とす():
    c = dict(CONTRACT, mode_signs={"lf": [1] * 6, "lb": [1] * 6, "rb": [1] * 6, "xx": [1] * 6})
    with pytest.raises(ValueError, match="ユニット名"):
        ModeAction(c, POSITIONS)


def test_符号の列名がモード名と食い違ったら落とす():
    c = dict(CONTRACT, mode_sign_columns=["fx", "fy", "tz", "fz", "tx", "tx"])
    with pytest.raises(ValueError, match="mode_sign_columns"):
        ModeAction(c, POSITIONS)


def test_ユニット順を入れ替えても同じユニットに同じ指令が出る():
    """符号表は名前で引く。ノード側の POSITIONS 順が変わっても対応が壊れない。"""
    a_ref = _drive(ModeAction(CONTRACT, POSITIONS), _rate(fx=1.0, tx=0.3), 20)
    swapped = ("rf", "rb", "lb", "lf")
    a_swp = _drive(ModeAction(CONTRACT, swapped), _rate(fx=1.0, tx=0.3), 20)
    for k, p in enumerate(POSITIONS):
        j = swapped.index(p)
        assert a_ref[k] == pytest.approx(a_swp[j]), f"{p}: servo がずれた"
        assert a_ref[4 + k] == pytest.approx(a_swp[4 + j]), f"{p}: esc がずれた"
