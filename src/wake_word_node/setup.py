from setuptools import setup
import os
from glob import glob

package_name = 'wake_word_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include the model file
        (os.path.join('share', package_name, 'resources'), 
         glob('resources/*.tflite')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Wake word detection node using TensorFlow Lite',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wake_word_node = wake_word_node.wake_word_node:main',
        ],
    },
)
