from setuptools import setup
import os
from glob import glob

package_name = 'autobot_mediapipe'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    (os.path.join('share','autobot_mediapipe','rviz'),glob(os.path.join('rviz','*.rviz*'))),
    (os.path.join('share',package_name,'launch'),glob(os.path.join('launch','*launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nx-ros2',
    maintainer_email='13377528435@sina.cn',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        '01_HandDetector = autobot_mediapipe.01_HandDetector:main',
        '02_PoseDetector = autobot_mediapipe.02_PoseDetector:main',
        '03_Holistic = autobot_mediapipe.03_Holistic:main',
        '04_FaceMesh = autobot_mediapipe.04_FaceMesh:main',
        '05_FaceEyeDetection = autobot_mediapipe.05_FaceEyeDetection:main',
        'test_msg = autobot_mediapipe.test_msg:main'
        ],
    },
)
