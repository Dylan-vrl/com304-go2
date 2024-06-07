import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'com304_go2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'models'), glob(os.path.join('models', '*.pth'))),
        (os.path.join('share', package_name, 'models', 'mono+stereo_640x192'), glob(os.path.join('models', 'mono+stereo_640x192', '*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dylan Vairoli',
    maintainer_email='dylan.vairoli@epfl.ch',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'go2_control_driver_node = com304_go2.go2_control_driver_node:main',
            'go2_control_node = com304_go2.go2_control_node:main',
            'go2_camera_node = com304_go2.go2_camera_node:main',
            'go2_autonomous_node = com304_go2.go2_autonomous_node:main',
        ],
    },
)
