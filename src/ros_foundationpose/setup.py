from setuptools import find_packages, setup
from glob import glob
package_name = 'ros_foundationpose'

packages = find_packages(exclude=['test'])  # 先获取自动找到的包列表
if f'{package_name}.utils' not in packages:  # 检查utils是否已包含
    packages.append(f'{package_name}.utils')  # 手动添加utils子包

setup(
    name=package_name,
    version='0.0.0',
    packages=packages,

    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name+'/launch', glob('launch/*.launch.py')),
        ('share/' + package_name+'/utils', glob('utils/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='3192937647@qq.com',
    description='TODO: Package description',
    license='Apache2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'foundationpose_node=ros_foundationpose.run_foundationpose:main',
            'take_photo_node=ros_foundationpose.take_photo:main',
            'delete_photo_node=ros_foundationpose.delete_photo:main'
        ],
    },
)
