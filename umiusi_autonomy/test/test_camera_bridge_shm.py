"""camera_bridge_node の shm socket の選び方。"""
import os

import pytest

pytest.importorskip("cv2")

from umiusi_autonomy.camera_bridge_node import _live_shm_socket  # noqa: E402


def test_無ければ指定のパス(tmp_path):
    p = str(tmp_path / "cam.sock")
    assert _live_shm_socket(p) == p


def test_古いsocketが残っていたら最も新しいものを選ぶ(tmp_path):
    """書き手が異常終了すると shmsink は <path>.0 に作り直す (GStreamer の挙動)。"""
    p = tmp_path / "cam.sock"
    p.touch()
    os.utime(p, (1, 1))
    (tmp_path / "cam.sock.0").touch()
    (tmp_path / "cam.sockX").touch()
    assert _live_shm_socket(str(p)) == str(p) + ".0"
