"""stack.launch.py の段の組み立てを固定する。

外し方はどれも実機でしか出ないので、ここで捕まえる:
  * RL が認識と同時に上がる -> 段を分けた意味 (起動時の CPU 競合の回避) が消える
  * 待ちが timeout で段を止める -> シグナルが来ないだけで起動が失敗する
  * 固定秒に戻る -> 速い機体では無駄に待ち、遅い機体では足りない
"""
import importlib.util
from pathlib import Path

import pytest

pytest.importorskip("launch")

from launch.actions import (  # noqa: E402
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)

LAUNCH = Path(__file__).resolve().parents[1] / "launch" / "stack.launch.py"


@pytest.fixture(scope="module")
def M():
    spec = importlib.util.spec_from_file_location("stack_launch", LAUNCH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(scope="module")
def ld(M):
    return M.generate_launch_description()


def _on_exit(handler):
    """RegisterEventHandler が起動する action の並び。describe() が公開 API。"""
    return handler.event_handler.describe()[1]


def _includes(entities):
    return [a for a in entities if isinstance(a, IncludeLaunchDescription)]


def _loc(inc):
    return str(inc.launch_description_source.location)


# --- 段の定義 (純粋なデータなので直接見る) -----------------------------------

def test_待ちは2段(M):
    assert [s["name"] for s in M.STAGES] == ["wait_control", "wait_perception"]


def test_IMUの待ちは常に行う(M):
    """use_control に関係なく待つ。IMU が流れていることが本当の前提条件で、
    誰が出すか (control か sim bridge か) は問わない。"""
    assert M.STAGES[0]["modes"] == (), "IMU の待ちに mode 条件が付いている"


def test_IMUはbest_effortで待つ(M):
    """センサ系の publisher は BEST_EFFORT が普通。RELIABLE では繋がらない。"""
    assert "--best-effort" in M.wait_args(M.STAGES[0])


def test_timeoutは従来のsleepを超えない(M):
    """シグナルで抜けるので通常は待たないが、上限が伸びると従来より遅くなりうる。"""
    assert M.STAGES[0]["timeout"] == "20"
    assert M.STAGES[1]["timeout"] == "35"


def test_待ちは段を止めない(M):
    """--allow-timeout が無いと、シグナルが来ないだけで起動が失敗する。"""
    for s in M.STAGES:
        assert "--allow-timeout" in M.wait_args(s), f"{s['name']} が timeout で段を止める"


# --- LaunchDescription の組み立て -------------------------------------------

def test_段の遷移はイベントで繋ぐ(ld):
    assert not [a for a in ld.entities if isinstance(a, TimerAction)], \
        "固定秒の待ちが復活している"
    assert len([a for a in ld.entities if isinstance(a, RegisterEventHandler)]) == 2


def test_RLは2つの起動点に分かれ二重起動しない(ld):
    """mode で起動点が違う。同じ action を両方の on_exit に渡すと二重に上がる。"""
    rl = []
    for h in (a for a in ld.entities if isinstance(a, RegisterEventHandler)):
        rl += [a for a in _includes(_on_exit(h)) if "rl_attitude" in _loc(a)]
    assert len(rl) == 2, f"RL の起動点が {len(rl)} 個 (2 個であるべき)"
    assert rl[0] is not rl[1], "同じ action を 2 箇所に渡している (二重起動になる)"


def test_認識の待ちの後にRLが上がる(ld):
    """full のときの本題: 検出器のロードが終わってから RL の torch を読む。"""
    handlers = [a for a in ld.entities if isinstance(a, RegisterEventHandler)]
    # wait_control 側は autonomy を含む。含まないほうが wait_perception 側
    after_percep = [h for h in handlers
                    if not any("core_autonomy" in _loc(a)
                               for a in _includes(_on_exit(h)))]
    assert len(after_percep) == 1
    assert any("rl_attitude" in _loc(a)
               for a in _includes(_on_exit(after_percep[0]))), \
        "認識の待ちの後に RL が上がらない"


def test_controlは最初に上げる(ld):
    """イベントの中ではなく LaunchDescription の直下 = 起動と同時。"""
    assert any("main.yaml" in _loc(a) for a in _includes(ld.entities))
