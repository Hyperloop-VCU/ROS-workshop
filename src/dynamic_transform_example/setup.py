from setuptools import find_packages, setup

package_name = 'dynamic_transform_example'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml', 'launch/display.launch.py', 'launch/rviz_config.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ray',
    maintainer_email='rayneralla@icloud.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "dynamic_transform_example = dynamic_transform_example.main:main"
        ],
    },
)
