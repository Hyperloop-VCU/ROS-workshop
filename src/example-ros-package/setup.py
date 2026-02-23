from setuptools import find_packages, setup

package_name = 'example_package'

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
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'pub_node = example_package.pub_node:main',
            'sub_node = example_package.sub_node:main',
        ],
    },
)
