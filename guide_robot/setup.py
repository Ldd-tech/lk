from setuptools import setup
import os
from glob import glob

package_name = 'guide_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 添加 launch 和 config 文件
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Guide robot for blind assistance',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_detector = guide_robot.obstacle_detector:main',
            'voice_announcer = guide_robot.voice_announcer:main',
            'object_detector = guide_robot.object_detector:main',
        ],
    },
)