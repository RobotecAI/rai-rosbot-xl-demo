import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rosbot_xl_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'params'), glob(os.path.join('params', '*.[yaml]*'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kdabrowski',
    maintainer_email='kacper.dabrowski@robotec.ai',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_spawner = rosbot_xl_demo.robot_spawner:main',
            'navigate_to_pose = rosbot_xl_demo.navigate_to_pose:main',
        ],
    },
)
