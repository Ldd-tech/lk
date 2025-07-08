from setuptools import setup
import os
from glob import glob

package_name = 'gps_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'pyproj', 'pyserial'],  # 添加这一行
    zip_safe=False,
    entry_points={
        'console_scripts': [
            'gps_driver = gps_pkg.gps_driver:main',
            'gps_converter = gps_pkg.gps_converter:main',
            'gps_to_odom = gps_pkg.gps_to_odom:main',
            'gps_visualizer = gps_pkg.gps_visualizer:main',
        ],
    }
)