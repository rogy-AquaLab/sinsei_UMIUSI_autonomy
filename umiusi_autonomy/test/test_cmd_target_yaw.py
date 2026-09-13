"""UI テレオペの旋回が効かない件の回帰テスト (known_issues B-17)。

固定するのは 2 つ:

  1. 旋回レートの積分が指令の送信レートに依存しないこと。
  2. **手順書が `ros2 param set` しろと書いた param を、そのノードが実際に受けること。**
     効かない set はエラーを返さないので、コードを読むかテストするしか気付く道が無い。
"""
import importlib.util
import re
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
NODE = ROOT / "umiusi_autonomy" / "umiusi_autonomy" / "classical_attitude_node.py"

# docs が「実行中に変えられる」と書いているノード -> ソース
NODE_SOURCES = {
    "/classical_attitude": NODE,
    "/navigator_node": ROOT / "umiusi_autonomy" / "umiusi_autonomy" / "navigator_node.py",
    "/perception_node": ROOT / "umiusi_autonomy" / "umiusi_autonomy" / "perception_node.py",
    "/rl_attitude_node": ROOT / "umiusi_rl_control" / "umiusi_rl_control" / "rl_attitude_node.py",
}


@pytest.fixture(scope="module")
def mod():
    """ROS のメッセージ型が要るので、無い環境では飛ばす (lint 環境など)。"""
    spec = importlib.util.spec_from_file_location("classical_attitude_node", NODE)
    m = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(m)
    except ImportError as e:
        pytest.skip(f"ROS の依存が無い: {e}")
    return m


def turn(mod, hz, seconds, rz=-0.2, scale=1.5, lead_max=1.05, dt_fixed=None):
    """指令を `hz` で送り続けたときに機体が回る角度 [rad] を返す。

    機体は目標に完全追従するものとする (追従が完璧なら先行量クランプは効かないので、
    積分そのものの性質だけが出る)。`dt_fixed` を与えると**修正前**の挙動になる。
    """
    yaw_sp, yaw_now, total = None, 0.0, 0.0
    for _ in range(int(hz * seconds)):
        yaw_sp = mod.advance_yaw_setpoint(
            yaw_sp, yaw_now, rz, scale, dt_fixed if dt_fixed else 1.0 / hz, lead_max)
        total += mod._wrap(yaw_sp - yaw_now)
        yaw_now = yaw_sp                      # 完全追従
    return total


@pytest.mark.parametrize("hz", [10.0, 30.0, 50.0, 100.0])
def test_旋回量が指令の送信レートに依存しない(mod, hz):
    """3 秒間 0.3 rad/s を指令したら、送信レートによらず 0.9 rad 回ること。"""
    got = turn(mod, hz, 3.0)
    assert got == pytest.approx(-0.9, abs=1e-9), f"{hz} Hz で {got:.3f} rad"


def test_修正前は送信レートぶんだけ遅かった(mod):
    """回帰の再現。UI 30 Hz・制御 50 Hz なら指定の 0.6 倍しか回らなかった。"""
    old = turn(mod, 30.0, 3.0, dt_fixed=1.0 / 50.0)
    assert old == pytest.approx(-0.9 * 0.6, abs=1e-9)      # 欠陥の再現
    assert turn(mod, 30.0, 3.0) == pytest.approx(-0.9, abs=1e-9)   # 修正後


def test_先行量は上限で頭打ちになる(mod):
    """機体が追随しないときに目標を積み続けない (誤差 180 度の安定平衡を避ける)。"""
    yaw_sp = None
    for _ in range(300):                       # 10 秒ぶん。放っておけば 3 rad 積む
        yaw_sp = mod.advance_yaw_setpoint(yaw_sp, 0.0, 1.0, 1.5, 1.0 / 30.0, 1.05)
    assert yaw_sp == pytest.approx(1.05, abs=1e-9)


def test_最初の指令は現在方位から始まる(mod):
    """溜めた目標が無いとき、機体の今の向きを起点にすること。"""
    got = mod.advance_yaw_setpoint(None, 1.0, 0.0, 1.5, 1.0 / 30.0, 1.05)
    assert got == pytest.approx(1.0, abs=1e-12)


def test_指令が途切れても方位目標が飛ばない(mod):
    """受信が止まった時間をまとめて積むと目標が飛ぶ。上限は 1 ステップぶんに抑える。"""
    assert 0.0 < mod.YAW_DT_MAX <= 0.5
    jump = abs(mod.advance_yaw_setpoint(0.0, 0.0, 1.0, 1.5, mod.YAW_DT_MAX, 1.05))
    assert jump <= 1.5 * mod.YAW_DT_MAX + 1e-12


# --- 手順書に書いた param が本当に実行中に効くか -----------------------------------


def documented_params():
    """docs / tools が `ros2 param set` しろと書いているもの -> {node: {param, ...}}。"""
    out = {}
    for path in list((ROOT / "docs").rglob("*.md")) + list((ROOT / "tools").rglob("*")):
        if path.is_dir() or path.suffix not in (".md", ".sh", ".py"):
            continue
        for node, param in re.findall(
                r"ros2 param set\s+(/[a-z_]+)\s+([a-z_][a-z0-9_]*)", path.read_text()):
            out.setdefault(node, set()).add(param)
    return out


def callback_body(source_path):
    """`add_on_set_parameters_callback` に渡している関数の本体テキスト。"""
    src = source_path.read_text()
    m = re.search(r"add_on_set_parameters_callback\(self\.(\w+)\)", src)
    if not m:
        return None
    body = re.search(rf"\n    def {m.group(1)}\(self, params\):(.*?)(?=\n    def )", src, re.S)
    return body.group(1) if body else None


def handled_params(source_path):
    """そのコールバックが**実際に分岐している** param 名。

    本文に名前が出てくるだけでは足りない (別の分岐のコメントや、ついでに読んでいる
    `get_parameter("...")` に一致してしまう)。`p.name` と突き合わせている名前だけを拾う。
    """
    body = callback_body(source_path)
    if body is None:
        return None
    names = set(re.findall(r'p\.name\s*==\s*"([a-z0-9_]+)"', body))
    for grp in re.findall(r"p\.name\s+in\s*\(([^)]*)\)", body):
        names |= set(re.findall(r'"([a-z0-9_]+)"', grp))
    return names


def test_手順書のparamが実行中に効く():
    """**set が成功を返すのに何も変わらない**のを防ぐ。

    現場は「param set したのに回らない」としか観測できず、符号や配線を疑って
    プールの時間を失う (2026-09-13 に実際に失った)。手順書に書いた名前は、
    そのノードのパラメータコールバックが必ず受けていること。
    """
    missing = []
    for node, params in documented_params().items():
        path = NODE_SOURCES.get(node)
        if path is None or not path.exists():
            continue
        handled = handled_params(path)
        assert handled is not None, f"{node} にパラメータコールバックが無い ({path.name})"
        missing += [f"{node} {p}" for p in sorted(params) if p not in handled]
    assert not missing, "手順書にあるが実行中に効かない: " + ", ".join(missing)
