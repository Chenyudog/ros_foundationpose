from setuptools import find_packages, setup
from glob import glob
package_name = 'ros_foundationpose'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name+'/launch', glob('launch/*.launch.py')),
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
