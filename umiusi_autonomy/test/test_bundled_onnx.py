"""models/detector の .onnx が隣の .pt と同じ出力を出すこと。

.pt を差し替えて .onnx を書き出し直し忘れると、perception_node は .onnx を使わずに
~/.cache へ書き出し直す (起動が遅くなり、Pi に onnx が無いと torch に戻る)。ここで捕まえる。
書き出し直しは umiusi_sim の tools/export_detector_onnx.py。

skip はテストの中で行う。モジュールの先頭で skip すると、ROS の pytest プラグイン下では
このディレクトリの他のテストまで集められなくなる。
"""
from pathlib import Path

import pytest

DETECTOR_DIR = Path(__file__).resolve().parents[1] / "models" / "detector"
ONNX = sorted(DETECTOR_DIR.glob("*.onnx"))


@pytest.fixture
def ld():
    pytest.importorskip("torch")
    pytest.importorskip("onnxruntime")
    mod = pytest.importorskip("umiusi_perception.learned_detector")
    if not hasattr(mod, "bundled_onnx_path"):
        pytest.skip("umiusi_perception の wheel が onnx 同梱に未対応")
    return mod


def test_the_default_detector_has_a_bundled_onnx():
    assert DETECTOR_DIR / "balloon_F320_20261003_320.onnx" in ONNX


@pytest.mark.parametrize("onnx", ONNX, ids=lambda p: p.name)
def test_bundled_onnx_matches_its_weights(ld, onnx, tmp_path, monkeypatch):
    monkeypatch.setenv("UMIUSI_ONNX_CACHE", str(tmp_path))  # 合わなければここに書き出される
    stem, size = onnx.stem.rsplit("_", 1)
    det = ld.load_learned_detector(str(DETECTOR_DIR / f"{stem}.pt"), input_size=int(size), backend="onnx")
    assert det.onnx_path == str(onnx)
