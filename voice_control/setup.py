from setuptools import setup
import os
from glob import glob

package_name = 'voice_control'

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
    maintainer='sunrise',
    maintainer_email='sunrise@example.com',
    description='Voice control for Wheeltec robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'voice_control_node = voice_control.voice_control_node:main',
            'speech_recognition_node = voice_control.speech_recognition_node:main'
        ],
    },
)