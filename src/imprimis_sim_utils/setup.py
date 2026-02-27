from setuptools import find_packages, setup

package_name = 'imprimis_sim_utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hyperlabs',
    maintainer_email='rayneralla@icloud.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'gps = imprimis_sim_utils.gps:main',
            'lidar = imprimis_sim_utils.lidar:main',
            'wait_for_tf = imprimis_sim_utils.wait_for_tf:main',
        ],
    },
)
