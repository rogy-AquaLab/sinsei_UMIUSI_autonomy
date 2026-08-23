import os
from glob import glob

from setuptools import find_packages, setup

package_name = "umiusi_rl_control"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ] + [
        # REP-103 ポリシーバンドル: export/ (素 torch) + golden.npz + meta.yaml。
        # SB3 の zip は同梱しない (実機の numpy 1.26 では読めず、export だけで動くため)
        entry
        for policy in ("av_cal1_best_rep103", "att_cal1_best_rep103", "av_sim2real2_rep103",
                       "av_cal5_3d_rep103")
        for entry in (
            (f"share/{package_name}/models/{policy}",
             [f for f in glob(f"models/{policy}/*") if os.path.isfile(f)]),
            (f"share/{package_name}/models/{policy}/export",
             glob(f"models/{policy}/export/*")),
        )
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="satoi",
    maintainer_email="satoimo3taro183@gmail.com",
    description=(
        "UMIUSI low-level control: a self-contained RL attitude(-velocity) controller, keyboard "
        "teleop, and arm/disarm (e-stop). Independent of the perception/autonomy stack."
    ),
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "rl_attitude_node = umiusi_rl_control.rl_attitude_node:main",
            "teleop_keyboard = umiusi_rl_control.teleop_keyboard:main",
        ],
    },
)
