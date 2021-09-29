import os
from glob import glob
from setuptools import setup

package_name = 'hal_allied_vision_camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'calibration'), glob('calibration/*.json')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ale_scar',
    maintainer_email='ale_scar@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        "calibration_node = hal_allied_vision_camera.camera_calibration:main",
        "av_node = hal_allied_vision_camera.allied_vision_node:main"
        ],
    },
)
