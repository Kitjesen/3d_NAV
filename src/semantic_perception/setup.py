from setuptools import setup, find_packages

package_name = "semantic_perception"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=[
        "setuptools",
        "numpy",
        "opencv-python",
        "open-clip-torch",
        "ultralytics",
    ],
    zip_safe=True,
    maintainer="3dNAV Team",
    maintainer_email="dev@3dnav.io",
    description="Semantic perception for VLN: YOLO-World + CLIP + ConceptGraphs scene graph",
    license="MIT",
    entry_points={
        "console_scripts": [
            "semantic_perception_node = semantic_perception.perception_node:main",
        ],
    },
)
