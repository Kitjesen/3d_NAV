from setuptools import find_packages, setup

package_name = "semantic_planner"

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
        "scipy",
        "openai",
        "anthropic",
        "dashscope",
        "jieba",
    ],
    extras_require={
        'agent': [
            'langchain-core>=0.2',
            'langchain-openai>=0.1',
            'langchain>=0.2',
            'pydantic>=2.0',
        ],
    },
    zip_safe=True,
    maintainer="3dNAV Team",
    maintainer_email="dev@3dnav.io",
    description="Semantic planner for VLN: Fast-Slow goal resolution + SayCan decomposition",
    license="MIT",
    entry_points={
        "console_scripts": [],
    },
)
