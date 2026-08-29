#!/usr/bin/env python3
"""検出器の 1 フレーム推論時間をスレッド数別に測る (ROS を介さない純粋な推論コスト)。

実機では他ノードと CPU を奪い合うため、**スレッドを増やすほど遅くなる**。
このツールでその逆転を確認できる (実測値は docs/performance_tuning.md)。

    NT=1 python3 infer_bench.py [checkpoint.pt]     # 実機は 1 が最速
    NT=4 python3 infer_bench.py                     # 単独実行なら 4 が最速

スタックを動かした状態と止めた状態の両方で回して比べること。
"""
from __future__ import annotations

import os
import sys
import time

import numpy as np


def run(nthreads: int, ckpt: str) -> None:
    import torch
    torch.set_num_threads(nthreads)
    from umiusi_perception.learned_detector import load_learned_detector

    det = load_learned_detector(ckpt)
    img = (np.random.rand(240, 320, 3) * 255).astype(np.uint8)
    for _ in range(3):          # ウォームアップ (初回は遅延 import と確保が入る)
        det(img)
    n, t0 = 20, time.time()
    for _ in range(n):
        det(img)
    dt = (time.time() - t0) / n
    print(f"  threads={nthreads}  1フレーム {dt*1000:6.1f} ms  -> 上限 {1/dt:5.2f} Hz")


if __name__ == "__main__":
    # 既定は同梱の検出器に合わせる (3 モデルとも同一アーキテクチャなので推論時間は変わらないが、
    # 計測対象が既定とずれていると読む側が混乱する)
    ckpt = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser(
        "~/ros2-ws/install/umiusi_autonomy/share/umiusi_autonomy/models/detector/camp_real2.pt")
    run(int(os.environ.get("NT", "4")), ckpt)
