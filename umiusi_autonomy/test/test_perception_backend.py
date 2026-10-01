import pytest

from umiusi_autonomy.perception_node import load_detector


class FakeLoader:
    """load_learned_detector の代わり。onnx を拒否するか、backend 引数を知らない古い wheel を真似る。"""

    def __init__(self, onnx_error=None, old_wheel=False):
        self.onnx_error = onnx_error
        self.old_wheel = old_wheel
        self.calls = []

    def __call__(self, path, **kwargs):
        self.calls.append(kwargs)
        if "backend" in kwargs:
            if self.old_wheel:
                raise TypeError("unexpected keyword argument 'backend'")
            if self.onnx_error:
                raise self.onnx_error
        return kwargs.get("backend", "torch")


def test_torch_does_not_pass_backend_so_old_wheels_keep_working():
    load = FakeLoader(old_wheel=True)
    assert load_detector(load, "m.pt", "torch", pytest.fail, input_size=256) == "torch"
    assert load.calls == [{"input_size": 256}]


def test_onnx_is_used_when_available():
    errors = []
    assert load_detector(FakeLoader(), "m.pt", "onnx", errors.append) == "onnx"
    assert errors == []


@pytest.mark.parametrize("loader", [
    FakeLoader(onnx_error=ModuleNotFoundError("No module named 'onnxruntime'")),
    FakeLoader(onnx_error=RuntimeError("export failed")),
    FakeLoader(old_wheel=True),
])
def test_onnx_failure_falls_back_to_torch_with_an_error(loader):
    errors = []
    assert load_detector(loader, "m.pt", "onnx", errors.append, input_size=320) == "torch"
    assert len(errors) == 1 and "falling back to torch" in errors[0]
    assert loader.calls[-1] == {"input_size": 320}  # 戻したときも他の設定は保つ


def test_unknown_backend_uses_torch():
    errors = []
    assert load_detector(FakeLoader(), "m.pt", "tensorrt", errors.append) == "torch"
    assert len(errors) == 1
