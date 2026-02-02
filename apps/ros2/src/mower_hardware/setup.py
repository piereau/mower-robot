from setuptools import find_packages, setup

package_name = 'mower_hardware'

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
    description='ROS 2 serial bridge for Mower Robot Arduino communication',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_bridge = mower_hardware.serial_bridge:main',
        ],
    },
)
