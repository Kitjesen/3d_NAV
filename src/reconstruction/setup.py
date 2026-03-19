from setuptools import setup, find_packages

package_name = "reconstruction"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "numpy", "scipy", "opencv-python"],
    zip_safe=True,
    maintainer="3dNAV Team",
    maintainer_email="dev@3dnav.io",
    description="三维重建节点 — 彩色语义点云聚合 + PLY 导出",
    license="MIT",
    entry_points={
        "console_scripts": [
            "reconstruction_node = reconstruction.reconstruction_node:main",
        ],
    },
)
