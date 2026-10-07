"""検出器の既定: 10/08 から balloon_F320_20261007 (濁った 10/03 の見え方を足した版)。旧モデルも残す。"""
import ast
from pathlib import Path

import yaml

PKG = Path(__file__).resolve().parents[1]


def test_default_model_is_camp_real_and_bundled():
    src = (PKG / "umiusi_autonomy" / "perception_node.py").read_text()
    names = [n.value for n in ast.walk(ast.parse(src))
             if isinstance(n, ast.Constant) and isinstance(n.value, str) and n.value.endswith(".pt")]
    assert names == ["balloon_F320_20261007.pt"]
    for name in ("balloon_F320_20261007.pt", "balloon_F320_20261003.pt", "balloon_F256_20261003.pt",
                 "camp_real.pt", "camp_real2.pt"):
        assert (PKG / "models" / "detector" / name).is_file(), name   # 旧モデルも残す


def test_min_confidence_is_set_for_the_deploy():
    cfg = yaml.safe_load((PKG / "config" / "autonomy.yaml").read_text())
    p = cfg["perception_node"]["ros__parameters"]
    assert p["min_confidence"] == 0.30          # F320_20261007 の運用値 (umiusi_sim の指定)
    assert p["min_confidence_red"] == 0.50      # 重りの赤誤検出が 3 フレーム続かない値 (10/03 15:19)


def test_per_colour_floor_overrides_the_common_floor():
    from types import SimpleNamespace as D

    from umiusi_autonomy.perception_node import filter_by_confidence
    dets = [D(colour="red", confidence=0.45), D(colour="red", confidence=0.55),
            D(colour="yellow", confidence=0.31), D(colour="blue", confidence=0.29)]
    kept = filter_by_confidence(dets, 0.30, {"red": 0.50, "yellow": -1.0, "blue": -1.0})
    assert [(d.colour, d.confidence) for d in kept] == [("red", 0.55), ("yellow", 0.31)]


def test_no_floor_keeps_everything():
    from types import SimpleNamespace as D

    from umiusi_autonomy.perception_node import filter_by_confidence
    dets = [D(colour="red", confidence=0.01)]
    assert filter_by_confidence(dets, -1.0, {"red": -1.0}) == dets
