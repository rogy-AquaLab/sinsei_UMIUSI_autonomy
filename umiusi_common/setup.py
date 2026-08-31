from setuptools import find_packages, setup

package_name = "umiusi_common"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="satoi",
    maintainer_email="satoimo3taro183@gmail.com",
    description="UMIUSI の層をまたいで共有する部品 (arm / imu_sanity)。ノードは持たない",
    license="MIT",
    tests_require=["pytest"],
    # entry_points は持たない。 ここはライブラリだけを置く場所で、ノードを足したくなったら
    # それは「層をまたいで共有する部品」ではないので、置き場所を間違えている合図になる。
)
