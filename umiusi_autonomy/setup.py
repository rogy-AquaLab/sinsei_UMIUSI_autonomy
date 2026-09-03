from glob import glob

from setuptools import find_packages, setup

package_name = "umiusi_autonomy"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        # 同梱の検出器。clone しただけで動かせるようにするため
        ("share/" + package_name + "/models/detector", glob("models/detector/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="satoi",
    maintainer_email="satoimo3taro183@gmail.com",
    description=(
        "Deploy-side ROS 2 nodes for UMIUSI balloon-popping autonomy: perception_node runs the "
        "learned detector on the onboard camera, navigator_node runs the shared behaviour FSM and "
        "drives the sinsei_umiusi_control thrusters. Thin rclpy wrappers around the ROS-free "
        "umiusi_perception package."
    ),
    license="MIT",
    entry_points={
        "console_scripts": [
            "perception_node = umiusi_autonomy.perception_node:main",
            "camera_bridge_node = umiusi_autonomy.camera_bridge_node:main",
            "navigator_node = umiusi_autonomy.navigator_node:main",
            "auto_target_generator = umiusi_autonomy.auto_target_generator:main",
            "wait_for_topic = umiusi_autonomy.wait_for_topic:main",
        ],
    },
)
