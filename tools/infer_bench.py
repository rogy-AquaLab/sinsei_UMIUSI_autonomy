#!/usr/bin/env python3
"""検出器の 1 フレーム推論時間を backend / スレッド数別に測る (ROS を介さない純粋な推論コスト)。

実機では他ノードと CPU を奪い合うため、**スレッドを増やすほど遅くなる**。
このツールでその逆転を確認できる (実測値は docs/performance_tuning.md)。

    NT=1 python3 infer_bench.py [checkpoint.pt]           # 実機は 1 が最速
    NT=4 python3 infer_bench.py                           # 単独実行なら 4 が最速
    NT=1 BACKENDS=onnx IMG=640x480 python3 infer_bench.py # backend と画像サイズを選ぶ

- BACKENDS: 空白区切り (既定 "torch onnx")。onnx が使えなければその行に理由を出して続ける
- IMG: 入力画像の 幅x高さ (既定は前カメラの 1280x720。前処理のリサイズは画像サイズで変わる)
- 内訳 (前処理 / モデル / decode) は umiusi_perception の wheel が onnx 対応版 (model 属性あり) のときだけ出る

スタックを動かした状態と止めた状態の両方で回して比べること。
"""
from __future__ import annotations

import os
import sys
import time

import numpy as np


def _median_ms(fn, n: int) -> float:
    ts = []
    for _ in range(n):
        t0 = time.perf_counter()
        fn()
        ts.append(time.perf_counter() - t0)
    return 1000.0 * float(np.median(ts))


def run(nthreads: int, ckpt: str, backend: str, img: np.ndarray, n: int = 20) -> None:
    import torch
    torch.set_num_threads(nthreads)
    from umiusi_perception import learned_detector as ld

    try:
        det = (ld.load_learned_detector(ckpt) if backend == "torch"
               else ld.load_learned_detector(ckpt, backend=backend))
    except Exception as e:  # noqa: BLE001
        print(f"  backend={backend:5s} 使えない ({type(e).__name__}: {e})")
        return
    for _ in range(3):          # ウォームアップ (初回は遅延 import と確保が入る)
        det(img)
    total = _median_ms(lambda: det(img), n)
    line = (f"  backend={backend:5s} threads={nthreads} input={getattr(det, 'input_size', '?')}  "
            f"1フレーム {total:6.1f} ms  -> 上限 {1000 / total:5.2f} Hz")
    model = getattr(det, "model", None)
    if model is not None:
        size = det.input_size
        x = ld.preprocess(img, size)
        with torch.no_grad():
            hm, wh = model(x)
            pre = _median_ms(lambda: ld.preprocess(img, size), n)
            inf = _median_ms(lambda: model(x), n)
        dec = _median_ms(lambda: ld.decode(hm[0], wh[0], img.shape[0], img.shape[1], size,
                                           conf_thresh=det.conf_thresh), n)
        line += f"  (前処理 {pre:.1f} / モデル {inf:.1f} / decode {dec:.1f})"
    print(line)


if __name__ == "__main__":
    # 既定は同梱の既定の検出器に合わせる (計測対象が既定とずれていると読む側が混乱する)
    ckpt = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser(
        "~/ros2-ws/install/umiusi_autonomy/share/umiusi_autonomy/models/detector/balloon_F320_20261003.pt")
    w, h = (int(v) for v in os.environ.get("IMG", "1280x720").split("x"))
    img = (np.random.default_rng(0).random((h, w, 3)) * 255).astype(np.uint8)
    print(f"{os.path.basename(ckpt)}  image {w}x{h}")
    for b in os.environ.get("BACKENDS", "torch onnx").split():
        run(int(os.environ.get("NT", "4")), ckpt, b, img)
