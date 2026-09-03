"""起動経路が 2 つある (tools/umiusi_stack.sh と launch/bringup.launch.py) ので、
段の定義がずれていないことを固定する。

片方だけ直すのが一番ありがちな壊し方で、実機でしか気付けない。ずれると
「シェルからは正しく待つが launch からは待たない」といった状態になる。
"""
import importlib.util
import re
from pathlib import Path

import pytest

pytest.importorskip("launch")

ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "tools" / "umiusi_stack.sh"
LAUNCH = ROOT / "umiusi_autonomy" / "launch" / "bringup.launch.py"


@pytest.fixture(scope="module")
def launch_mod():
    spec = importlib.util.spec_from_file_location("stack_launch", LAUNCH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(scope="module")
def stages(launch_mod):
    return {s["topic"]: s for s in launch_mod.STAGES}


@pytest.fixture(scope="module")
def shell_waits():
    """シェル側の wait_topic 呼び出し -> {topic: (timeout, best_effort)}。"""
    out = {}
    for m in re.finditer(r"^\s*wait_topic\s+(\S+)\s+(\d+)([^\n]*)$",
                         SCRIPT.read_text(encoding="utf-8"), re.M):
        topic, timeout, rest = m.group(1), m.group(2), m.group(3)
        out[topic] = (timeout, "--best-effort" in rest)
    return out


def test_シェルが待つトピックはlaunchの段と同じ(shell_waits, stages):
    assert set(shell_waits) == set(stages), (
        f"シェル {sorted(shell_waits)} と launch {sorted(stages)} がずれている")


def test_timeoutが両者で一致する(shell_waits, stages):
    for topic, (timeout, _) in shell_waits.items():
        assert timeout == stages[topic]["timeout"], (
            f"{topic}: シェル {timeout}s / launch {stages[topic]['timeout']}s")


def test_best_effortの指定が両者で一致する(shell_waits, stages):
    for topic, (_, best_effort) in shell_waits.items():
        assert best_effort == stages[topic]["best_effort"], (
            f"{topic}: シェル best_effort={best_effort} / "
            f"launch {stages[topic]['best_effort']}")


# --- ログ待ちを増やさない ----------------------------------------------------

def test_ログで待つのはrlだけ():
    """ログ経由の待ちは、背景プロセスの truncate と競合しうる (rl 以外は
    トピックで待てる)。増えていたら、そちらもトピックにできないか考えること。"""
    calls = re.findall(r"^\s*wait_log\s+\"([^\"]+)\"",
                       SCRIPT.read_text(encoding="utf-8"), re.M)
    assert calls == ["$LOGDIR/rl.log"], f"ログ待ちが増えている: {calls}"


def test_ログで待つ前に必ず親で空にする():
    """背景側の > は fork 後に効くので、前回の完了行を拾う。

    「どこかに truncate がある」では不十分で、wait_log より前になければ意味が無い。
    """
    lines = SCRIPT.read_text(encoding="utf-8").splitlines()
    for i, line in enumerate(lines):
        m = re.match(r"^\s*wait_log\s+\"([^\"]+)\"", line)
        if not m:
            continue
        log = m.group(1)
        before = [j for j, ln in enumerate(lines[:i])
                  if re.match(r"^\s*: > \"" + re.escape(log) + r"\"", ln)]
        assert before, f"{log} を wait_log ({i + 1} 行目) で読む前に空にしていない"


def test_認識を上げるmodeでは必ず待つ(stages, launch_mod):
    """段の待ちと include の条件が別々だと「上がるのに待たない」mode ができる。
    同じ定数を共有していること (別々に書かれていないこと) を固定する。"""
    percep = stages["/perception_node/detections"]
    assert percep["modes"] is launch_mod.PERCEPTION_MODES, \
        "認識の待ちが PERCEPTION_MODES と別物になっている (include の条件とずれる)"
    assert "perception" in percep["modes"], "mode=perception で認識の完了を待たない"
