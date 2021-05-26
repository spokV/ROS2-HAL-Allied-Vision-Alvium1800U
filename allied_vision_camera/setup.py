from setuptools import setup

package_name = 'allied_vision_camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
        "camera_node = allied_vision_camera.camera_node:main",
        "calibration_node = allied_vision_camera.camera_calibration:main",
        "aruco_detector = allied_vision_camera.aruco_pose_estimator:main",
        "pose_filter = allied_vision_camera.pose_filt_cmd:main",
        "av_node = allied_vision_camera.allied_vision_node:main"
        ],
    },
)
