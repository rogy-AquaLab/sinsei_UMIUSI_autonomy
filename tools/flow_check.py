#!/usr/bin/env python3
"""カメラ映像でオプティカルフローが**実際に出るか**をその場で判定する。

方策は横方向の速度を観測できず、定常 sway はどんな罰でも直せない (known_issues)。
解消には速度の観測が要り、その第一候補が下向きカメラ (cam2) のオプティカルフロー。
**ただし水中では成立しないことがある。** 手元の水中実写 (`ai/水中赤風船近づき_深度RGB.bag`,
RealSense 1280x720) で測ると:

    特徴点 300/300 が「追跡成功」、しかし前後誤差 2.6 px / フロー 5.5 px = 0.48、
    FB<1px は 36% だけ。コントラスト std は 10〜16/255。

**追跡成功率は当てにならない。** 無地の画像でも LK は勾配ノイズを追跡して「成功」を返す。
順方向に追って逆方向に追い戻し、元の点に戻るか (forward-backward 誤差) が本当の検定。

    python3 flow_check.py --device /dev/video4        # 下向きカメラを直接
    python3 flow_check.py --topic /front_cam/image_raw # ROS トピックから
    python3 flow_check.py --video path/to.mp4          # 録った映像を後から

判定の基準 (上の実測から置いた):
  * コントラスト std >= 25    … これ未満はテクスチャが無い。露光ではなく**濁り/曇り**
  * FB<1px の割合 >= 70%      … フローを速度に使える下限
  * FB 誤差 / フロー <= 0.2   … 誤差がフローに対して十分小さい

ダメなときの順番: **レンズの曇り/汚れを拭く -> プール底 (タイル・ライン) に向ける ->
照明を足す -> 露光を詰める。** 露光を触るのは最後。コントラストが無い画像は露光では直らない。
"""

from __future__ import annotations

import argparse
import sys

import cv2
import numpy as np

GF = dict(maxCorners=300, qualityLevel=0.01, minDistance=8, blockSize=7)
LK = dict(winSize=(21, 21), maxLevel=3,
          criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
CONTRAST_MIN = 25.0
FB_GOOD_FRAC = 0.70
FB_RATIO_MAX = 0.20


def measure(g0, g1):
    """1 組のフレームから (特徴点数, フロー中央値, FB 誤差中央値, FB<1px 割合)。"""
    p0 = cv2.goodFeaturesToTrack(g0, mask=None, **GF)
    if p0 is None or len(p0) < 10:
        return 0, float("nan"), float("nan"), 0.0
    p1, st1, _ = cv2.calcOpticalFlowPyrLK(g0, g1, p0, None, **LK)
    p0b, st2, _ = cv2.calcOpticalFlowPyrLK(g1, g0, p1, None, **LK)
    ok = (st1.ravel() == 1) & (st2.ravel() == 1)
    if ok.sum() < 10:
        return len(p0), float("nan"), float("nan"), 0.0
    fl = np.linalg.norm((p1 - p0).reshape(-1, 2)[ok], axis=1)
    fb = np.linalg.norm((p0b - p0).reshape(-1, 2)[ok], axis=1)
    return len(p0), float(np.median(fl)), float(np.median(fb)), float((fb < 1.0).mean())


def verdict(contrast, flow, fb, good):
    """3 つの基準で go / no-go と、**次に何をすべきか**を返す。"""
    if contrast < CONTRAST_MIN:
        return ("NG", f"コントラスト {contrast:.1f} < {CONTRAST_MIN:.0f}: テクスチャが無い。"
                      "レンズの曇り/汚れを拭き、プール底 (タイル・ライン) に向けること。"
                      "**露光を触るのは最後**")
    if not np.isfinite(flow) or flow < 0.3:
        return ("?", "フローがほぼ 0: 機体が動いていないか、視野が遠すぎる。"
                     "ゆっくり並進させながら測ること")
    ratio = fb / flow
    if good < FB_GOOD_FRAC or ratio > FB_RATIO_MAX:
        return ("NG", f"FB<1px {good*100:.0f}% (>= {FB_GOOD_FRAC*100:.0f}% 必要) / "
                      f"FB誤差比 {ratio:.2f} (<= {FB_RATIO_MAX} 必要): "
                      "追跡が信用できない。テクスチャを増やすか露光を短くする")
    return ("OK", f"FB<1px {good*100:.0f}% / FB誤差比 {ratio:.2f}: 速度推定に使える")


def frames_from_topic(topic, n, timeout):
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.node import Node
    from sensor_msgs.msg import CompressedImage, Image

    comp = topic.endswith("/compressed")
    got, bridge = [], CvBridge()

    class Sub(Node):
        def __init__(self):
            super().__init__("flow_check")
            self.create_subscription(CompressedImage if comp else Image, topic,
                                     self._cb, 10)

        def _cb(self, m):
            got.append(bridge.compressed_imgmsg_to_cv2(m, "bgr8") if comp
                       else bridge.imgmsg_to_cv2(m, "bgr8"))

    rclpy.init()
    node = Sub()
    import time
    t0 = time.time()
    while len(got) < n and time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
    rclpy.shutdown()
    return got


def main() -> int:
    ap = argparse.ArgumentParser()
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--device", help="V4L2 デバイス (下向きカメラは既定 /dev/video4)")
    src.add_argument("--topic", help="sensor_msgs/Image または .../compressed のトピック")
    src.add_argument("--video", help="録画ファイル")
    ap.add_argument("--frames", type=int, default=60, help="評価するフレーム数")
    ap.add_argument("--timeout", type=float, default=20.0)
    ap.add_argument("--save", default="", help="代表フレームの保存先 (JPEG)")
    args = ap.parse_args()

    if args.topic:
        imgs = frames_from_topic(args.topic, args.frames, args.timeout)
        if len(imgs) < 2:
            print(f"{args.topic} から画像が来ない ({len(imgs)} 枚)。"
                  "カメラノードが上がっているか、トピック名と QoS を確認すること")
            return 1
    else:
        cap = cv2.VideoCapture(args.device if args.device else args.video)
        if not cap.isOpened():
            print(f"開けない: {args.device or args.video}")
            return 1
        imgs = []
        while len(imgs) < args.frames:
            ok, im = cap.read()
            if not ok:
                break
            imgs.append(im)
        cap.release()
        if len(imgs) < 2:
            print(f"フレームが読めない ({len(imgs)} 枚)")
            return 1

    grays = [cv2.cvtColor(i, cv2.COLOR_BGR2GRAY) for i in imgs]
    h, w = grays[0].shape
    print(f"{len(imgs)} フレーム {w}x{h}\n")
    print("  #   特徴点  フロー[px]  FB誤差[px]  FB<1px  コントラスト  鮮鋭度")
    rows = []
    step = max(1, len(grays) // 12)
    for i in range(0, len(grays) - 1, step):
        n, fl, fb, good = measure(grays[i], grays[i + 1])
        c = float(grays[i].std())
        s = float(cv2.Laplacian(grays[i], cv2.CV_64F).var())
        rows.append((n, fl, fb, good, c, s))
        fls = f"{fl:10.2f}" if np.isfinite(fl) else "         -"
        fbs = f"{fb:10.2f}" if np.isfinite(fb) else "         -"
        print(f"  {i:3} {n:7} {fls} {fbs} {good*100:6.0f}% {c:12.1f} {s:8.1f}")

    a = np.array([(r[1], r[2], r[3], r[4]) for r in rows if np.isfinite(r[1])])
    if not len(a):
        print("\n有効な測定が無い。映像が静止しているか、特徴点が取れていない")
        return 1
    flow, fb, good, contrast = a.mean(axis=0)
    print(f"\n  平均: フロー {flow:.2f} px  FB誤差 {fb:.2f} px  "
          f"FB<1px {good*100:.0f}%  コントラスト {contrast:.1f}")
    tag, why = verdict(contrast, flow, fb, good)
    print(f"\n  判定: **{tag}** — {why}")
    print(f"\n  参考: 手元の水中実写 (濁りが強い) は コントラスト 10〜16 / FB<1px 36% で NG。"
          f"\n  cap 0.25 の上限速度 0.17 m/s・高度 1 m ならフローは約 2.4 px/frame @50 Hz。")
    if args.save and imgs:
        cv2.imwrite(args.save, imgs[len(imgs) // 2], [cv2.IMWRITE_JPEG_QUALITY, 85])
        print(f"  代表フレーム: {args.save}")
    return 0 if tag == "OK" else 1


if __name__ == "__main__":
    sys.exit(main())
