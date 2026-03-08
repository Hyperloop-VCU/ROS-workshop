from setuptools import find_packages, setup

package_name = 'imu_example'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'launch/both_nodes.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ray',
    maintainer_email='rayneralla@icloud.com',
    description='Example package demonstrating IMU data communication via ROS topics',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'imu_publisher = imu_example.imu_publisher:main',
            'imu_subscriber = imu_example.imu_subscriber:main',
        ],
    },
)
