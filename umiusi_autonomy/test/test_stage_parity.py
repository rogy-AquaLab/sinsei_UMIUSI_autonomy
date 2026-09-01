"""起動経路が 2 つある (tools/umiusi_stack.sh と launch/stack.launch.py) ので、
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
LAUNCH = ROOT / "umiusi_autonomy" / "launch" / "stack.launch.py"


@pytest.fixture(scope="module")
def stages():
    spec = importlib.util.spec_from_file_location("stack_launch", LAUNCH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return {s["topic"]: s for s in mod.STAGES}


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
    """背景側の > は fork 後に効くので、前回の完了行を拾う。"""
    text = SCRIPT.read_text(encoding="utf-8")
    for log in re.findall(r"^\s*wait_log\s+\"([^\"]+)\"", text, re.M):
        assert re.search(r"^\s*: > \"" + re.escape(log) + r"\"", text, re.M), \
            f"{log} を wait_log で読む前に空にしていない"
