"""推力の向き (基ごとの duty の符号) を **control の設定から取ってくる**。

## なぜ取りに行くのか

極性の置き場が 2 つに割れている:

  * control `params/controllers.yaml` の `thruster_controller_<pos>.is_forward`
    — ただしこれが効くのは `logic/thruster/linear_acceleration.hpp` の経路だけ
    (`sign = is_forward ? 1.0 : -1.0` を duty に掛ける)。
  * autonomy が使う `/cmd/direct/...` は **`duty_cycle` を素通しする**
    (`thruster_controller.cpp` の direct 購読は msg の値をそのまま
    `esc_duty_cycle` へ書く)。max_duty もスルーレートも効かない (known_issues B-12)。

つまり **control の yaml で極性を直しても autonomy には届かない。** 2 箇所に同じ値を
書くと必ず片方が腐るので、**control を唯一の正として autonomy が合わせに行く。**

`is_forward` は「その基の正の指令が順方向の推力になるか」で、ここで欲しい符号と
意味が同じなので、そのまま使える (true -> +1.0 / false -> -1.0)。

## 取れないとき

control が上がっていない構成 (umiusi_sim_bridge で sim に繋ぐ、ベンチで
autonomy だけ動かす) では取れない。そのときは**ノード自身の `thrust_sign` に落ちる** —
黙って落ちるのではなく警告を出す。「control で直したのに効いていない」と
「そもそも control が居ない」は現場で区別が付かないと詰む。
"""

from __future__ import annotations

from rcl_interfaces.srv import GetParameters

CONTROLLER_NS = "/thruster_controller_"
PARAM = "is_forward"
PARAM_DISABLED = "esc_disabled"


def _read_bool(node, positions, param, timeout_sec=3.0):
    """control の `thruster_controller_<pos>` から bool パラメータを読む。

    返り値は (values, detail)。**1 基でも取れなければ None** — 一部だけ control 由来という
    混ざった状態が一番危ない (どちらの値で動いているのかログから追えない)。
    """
    import rclpy
    out, notes = [], []
    for p in positions:
        srv = f"{CONTROLLER_NS}{p}/get_parameters"
        cli = node.create_client(GetParameters, srv)
        try:
            if not cli.wait_for_service(timeout_sec=timeout_sec):
                return None, f"{srv} が応答しない (control が上がっていない)"
            req = GetParameters.Request()
            req.names = [param]
            fut = cli.call_async(req)
            rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout_sec)
            res = fut.result()
            if res is None or not res.values:
                return None, f"{srv} が {param} を返さない"
            v = res.values[0]
            if v.type != 1:                      # PARAMETER_BOOL
                return None, f"{p}: {param} が bool でない (type={v.type})"
            out.append(bool(v.bool_value))
            notes.append(f"{p}={'T' if v.bool_value else 'F'}")
        finally:
            node.destroy_client(cli)
    return out, f"control から取得 ({param}): " + " ".join(notes)


def live_from_control(node, positions, fallback, timeout_sec=3.0):
    """control の `esc_disabled` から「生きているスラスタ」を決める。

    **`esc_disabled` は `/cmd/direct` 経路にも効く** (`thruster_controller.cpp` の direct
    購読が `resolve_thruster_mode(esc_disabled, ...)` を通す) ので、control でスラスタを
    殺すと autonomy が何も知らないまま「死んだ基に配分し続ける」状態になる。解いた
    wrench と実際に出る力が食い違い、残りの基が誤った前提で釣り合いを取る。
    **だから control を唯一の正として読みに行く。**
    """
    vals, detail = _read_bool(node, positions, PARAM_DISABLED, timeout_sec)
    if vals is None:
        node.get_logger().warning(
            f"control から {PARAM_DISABLED} を取得できない ({detail})。"
            f"ノード自身の live_thrusters={fallback} を使う")
        return list(fallback)
    live = [not d for d in vals]
    dead = [p for p, a in zip(positions, live) if not a]
    node.get_logger().info(f"live_thrusters={live} ({detail})")
    if dead:
        node.get_logger().warning(
            f"**control が {dead} を esc_disabled にしている。** 配分から外す。"
            "残りで 6 自由度は保つが余裕は無い")
    return live


def from_control(node, positions, timeout_sec=3.0):
    """control の `thruster_controller_<pos>` から `is_forward` を読む。

    返り値は (signs, detail)。**1 基でも取れなければ signs は None** — 一部だけ
    control 由来・残りは既定、という混ざった状態がいちばん危ない (どちらの値で
    動いているのかログから追えなくなる)。detail は人が読むための経緯。
    """
    signs, notes = [], []
    for p in positions:
        srv = f"{CONTROLLER_NS}{p}/get_parameters"
        cli = node.create_client(GetParameters, srv)
        try:
            if not cli.wait_for_service(timeout_sec=timeout_sec):
                return None, f"{srv} が応答しない (control が上がっていない)"
            req = GetParameters.Request()
            req.names = [PARAM]
            fut = cli.call_async(req)
            import rclpy
            rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout_sec)
            res = fut.result()
            if res is None or not res.values:
                return None, f"{srv} が {PARAM} を返さない"
            v = res.values[0]
            # PARAMETER_BOOL = 1。型が違うなら設定ミスなので混ぜずに諦める
            if v.type != 1:
                return None, f"{p}: {PARAM} が bool でない (type={v.type})"
            signs.append(1.0 if v.bool_value else -1.0)
            notes.append(f"{p}={'順' if v.bool_value else '逆'}")
        finally:
            node.destroy_client(cli)
    return signs, "control から取得: " + " ".join(notes)


def resolve(node, positions, source, fallback):
    """`thrust_sign_source` に従って符号を決め、**必ずログに残す**。

    source は "control" (control の is_forward に合わせる) か "param"
    (ノード自身の `thrust_sign` を使う)。どちらで動いているかは、実験の
    やり直しを避けるために必ず起動ログへ出す。
    """
    if source == "param":
        node.get_logger().info(f"thrust_sign={fallback} (source=param)")
        return list(fallback)
    if source != "control":
        node.get_logger().warning(
            f"thrust_sign_source='{source}' は不明。'control' として扱う")
    signs, detail = from_control(node, positions)
    if signs is None:
        node.get_logger().warning(
            f"control から推力の向きを取得できない ({detail})。"
            f"ノード自身の thrust_sign={fallback} に落ちる。"
            "**control の yaml をここで直しても効かない**ので、"
            "sim bridge / ベンチで動かしているのでなければ control の起動を確認すること")
        return list(fallback)
    node.get_logger().info(f"thrust_sign={signs} ({detail})")
    if any(s < 0 for s in signs):
        node.get_logger().warning(
            "推力を反転させている基がある。control の is_forward に合わせた結果で、"
            "これが意図と違うなら control 側の params/controllers.yaml を直すこと")
    return signs
