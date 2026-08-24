"""同梱ポリシーバンドルの配備前検証 (issue #15 A-5) をテストとしても回す。

rl_attitude_node が読み込み時にやる検証と同じ: export/ を PolicyRunner (実機の推論経路) で
読み、frame 契約 (rep103) と観測次元を確認し、golden.npz (sim で記録した観測→行動ペア) を
再生して一致を要求する。これが通れば、重み・正規化統計・観測レイアウト・frame 規約の
すべてが sim の検証済み状態と一致している。
"""
from pathlib import Path

import numpy as np
import pytest

torch = pytest.importorskip("torch")

from umiusi_rl_control.policy_infer import PolicyRunner  # noqa: E402

MODELS = Path(__file__).resolve().parent.parent / "models"
BUNDLES = sorted(p.name for p in MODELS.iterdir() if (p / "export").is_dir())


def test_bundles_present():
    assert "av_cal1_best_rep103" in BUNDLES      # 既定ポリシーは必ず同梱
    assert "att_cal1_best_rep103" in BUNDLES     # 姿勢のみフォールバック


@pytest.mark.parametrize("name", BUNDLES)
def test_bundle_contract_and_golden(name):
    d = MODELS / name
    runner = PolicyRunner(d / "export")
    assert runner.meta.get("obs_frame") == "rep103", name
    assert runner.obs_dim in (14, 17), (name, runner.obs_dim)
    # 鉛直指令インターロックの契約: vertical_ok は 3-D vectoring 系だけが持つ。
    # 水平専用ポリシーに付くとノードのクランプが外れて転覆リスクになる
    assert runner.meta.get("vertical_ok", False) == ("3d" in name), name

    g = np.load(d / "golden.npz")
    assert int(g["obs_dim"]) == runner.obs_dim, name
    worst = max(float(np.abs(runner.act(o) - a).max())
                for o, a in zip(g["obs"], g["act"]))
    assert worst < 1e-4, f"{name}: max |action - golden| = {worst:.2e}"
