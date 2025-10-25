from setuptools import setup

package_name = 'live_agent'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/live_agent.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='ROS2 node for Deep Agents integration with LangChain',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'live_agent = live_agent.live_agent:main',
        ],
    },
)
