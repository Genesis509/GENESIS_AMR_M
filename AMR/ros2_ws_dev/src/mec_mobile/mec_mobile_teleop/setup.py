from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mec_mobile_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='genesis',
    maintainer_email='jacquetdaniel19@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'xbox_teleop = mec_mobile_teleop.xbox_teleop:main',
            'camera_subscriber = mec_mobile_teleop.camera_subscriber:main',
        ],
    },
)
