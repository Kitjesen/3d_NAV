from setuptools import setup

package_name = 'nav_rings'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='inovxio',
    maintainer_email='dev@inovxio.com',
    description='Three-ring cognitive architecture nodes',
    license='MIT',
    entry_points={
        'console_scripts': [
            'safety_monitor = nav_rings.safety_monitor:main',
            'evaluator = nav_rings.evaluator:main',
            'dialogue_manager = nav_rings.dialogue_manager:main',
        ],
    },
)
