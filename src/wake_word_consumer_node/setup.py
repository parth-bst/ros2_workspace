from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'wake_word_consumer_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='ROS2 node for consuming wake word detection results and LLM responses on Raspberry Pi',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wake_word_consumer_node = wake_word_consumer_node.wake_word_consumer_node:main',
        ],
    },
)
