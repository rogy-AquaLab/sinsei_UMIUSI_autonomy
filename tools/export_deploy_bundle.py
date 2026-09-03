#!/usr/bin/env python3
"""バンドル (export/) を C++ から読める 1 ファイルの TorchScript に固める。

C++ 側には JSON パーサも npz リーダも無い。ファイルを 3 つ読ませる代わりに、
ネットと正規化パラメータと action_contract を 1 つの TorchScript に入れる。

    python3 tools/export_deploy_bundle.py umiusi_rl_control/models/av_cal1_best_rep103

-> そのバンドルの直下に deploy.pt ができる。

**元のバンドルは変えない。** sim の export 契約に手を入れずに済むよう、変換は配備側で行う。
バンドルを差し替えたらこれを流し直すこと。

入っているもの:
  forward(x)     ネット本体 (正規化前の観測を渡す。正規化はこの中でやらない)
  obs_mean/var   正規化パラメータ (obs_norm.npz と同じ値)
  obs_clip/eps
  obs_dim / act_dim
  action_mode    "direct" か "modes"
  thrust_per_cmd / thrust_curve_exp / servo_range_deg   ← action_contract がある場合のみ

golden.npz はここには入れない。配備前検証は別に流す (検証用のデータを配備物に混ぜない)。
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import torch


def _load_net(export: Path):
    """policy_infer と同じ手順でネットを組む。ここで独自に組み直すと静かにずれる。"""
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "umiusi_rl_control"))
    from umiusi_rl_control.policy_infer import PolicyRunner

    return PolicyRunner(str(export))


class _Deploy(torch.nn.Module):
    """属性は __init__ で登録し、torch.jit.script で固めること。
    trace はネットの実行だけを写すので、属性が落ちる。"""

    def __init__(self, net, norm, meta, obs_dim):
        super().__init__()
        self.net = net
        self.register_buffer("obs_mean", torch.tensor(norm["mean"], dtype=torch.float64))
        self.register_buffer("obs_var", torch.tensor(norm["var"], dtype=torch.float64))
        self.obs_clip = float(norm["clip"])
        self.obs_eps = float(norm["eps"])
        self.obs_dim = int(obs_dim)
        self.act_dim = int(meta.get("act_dim", 0))
        self.action_mode = str(meta.get("action_mode", "direct"))
        # contract が無いバンドル (direct 出力) では 0。読む側は action_mode で分岐する
        c = meta.get("action_contract") or {}
        self.thrust_per_cmd = float(c.get("thrust_per_cmd", 0.0))
        self.thrust_curve_exp = float(c.get("thrust_curve_exp", 0.0))
        self.servo_range_deg = float(c.get("servo_range_deg", 0.0))

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.net(x)


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("bundle", help="models/<name> (export/ を含むディレクトリ)")
    p.add_argument("-o", "--out", default=None, help="出力先 (既定: <bundle>/deploy.pt)")
    a = p.parse_args(argv)

    bundle = Path(a.bundle)
    export = bundle / "export"
    if not export.is_dir():
        p.error(f"{export} がありません")

    torch.set_num_threads(1)
    runner = _load_net(export)
    meta = json.loads((export / "meta.json").read_text(encoding="utf-8"))
    norm = np.load(export / "obs_norm.npz")

    mod = _Deploy(runner.net, norm, meta, runner.obs_dim)
    # trace ではなく script。trace はネットの実行だけを写し、属性を落とす
    m = torch.jit.script(mod)

    out = Path(a.out) if a.out else bundle / "deploy.pt"
    m.save(str(out))
    print(f"{out}  obs {mod.obs_dim} -> act {mod.act_dim}  mode={mod.action_mode}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
