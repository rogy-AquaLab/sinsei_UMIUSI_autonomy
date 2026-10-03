"""検出器の既定 (B-24): 10/03 から balloon_F320。camp_real2 は 10/01 の映像で赤い風船を取れなかった。"""
import ast
from pathlib import Path

import yaml

PKG = Path(__file__).resolve().parents[1]


def test_default_model_is_camp_real_and_bundled():
    src = (PKG / "umiusi_autonomy" / "perception_node.py").read_text()
    names = [n.value for n in ast.walk(ast.parse(src))
             if isinstance(n, ast.Constant) and isinstance(n.value, str) and n.value.endswith(".pt")]
    assert names == ["balloon_F320_20261003.pt"]
    for name in ("balloon_F320_20261003.pt", "balloon_F256_20261003.pt", "camp_real.pt", "camp_real2.pt"):
        assert (PKG / "models" / "detector" / name).is_file(), name   # 旧モデルも残す


def test_min_confidence_is_set_for_the_deploy():
    cfg = yaml.safe_load((PKG / "config" / "autonomy.yaml").read_text())
    assert cfg["perception_node"]["ros__parameters"]["min_confidence"] == 0.40
