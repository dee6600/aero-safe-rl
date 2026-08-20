from setuptools import find_packages, setup

package_name = 'aero_bridge'

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
    maintainer='dee6600',
    maintainer_email='durgeshreddiyar@gmail.com',
    description='PX4 <-> ROS 2 telemetry and command bridge for aero-safe-rl',
    license='TBD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_flight = aero_bridge.test_flight:main',
        ],
    },
)
