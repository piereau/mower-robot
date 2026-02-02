from setuptools import find_packages, setup

package_name = 'mower_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pierrot',
    maintainer_email='pierrot@example.com',
    description='ROS 2 WebSocket bridge for teleoperation commands',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ws_bridge = mower_teleop.ws_bridge:main',
        ],
    },
)
