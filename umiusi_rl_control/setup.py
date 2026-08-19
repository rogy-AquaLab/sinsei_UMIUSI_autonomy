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
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        # ディレクトリを除外する (export/ サブディレクトリを含むため)
        ("share/" + package_name + "/models/cruise_policy",
         [f for f in glob("models/cruise_policy/*") if os.path.isfile(f)]),
        # SB3 非依存の書き出し (実機の numpy 1.26 では policy zip が読めないため)
        ("share/" + package_name + "/models/cruise_policy/export",
         glob("models/cruise_policy/export/*")),
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
