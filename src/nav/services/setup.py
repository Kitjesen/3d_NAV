from setuptools import setup

package_name = 'nav_services'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kitjesen',
    maintainer_email='hong@inovxio.com',
    description='L5 运营服务 — 地图管理、巡检路线、电子围栏、任务调度、任务日志',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [],
    },
)
