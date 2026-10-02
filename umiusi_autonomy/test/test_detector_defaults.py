"""検出器の既定 (B-24): camp_real2 は 10/01 の映像で赤い風船を取れなかったので使わない。"""
import ast
from pathlib import Path

import yaml

PKG = Path(__file__).resolve().parents[1]


def test_default_model_is_camp_real_and_bundled():
    src = (PKG / "umiusi_autonomy" / "perception_node.py").read_text()
    names = [n.value for n in ast.walk(ast.parse(src))
             if isinstance(n, ast.Constant) and isinstance(n.value, str) and n.value.endswith(".pt")]
    assert names == ["camp_real.pt"]
    assert (PKG / "models" / "detector" / "camp_real.pt").is_file()


def test_min_confidence_is_set_for_the_deploy():
    cfg = yaml.safe_load((PKG / "config" / "autonomy.yaml").read_text())
    assert cfg["perception_node"]["ros__parameters"]["min_confidence"] == 0.45
