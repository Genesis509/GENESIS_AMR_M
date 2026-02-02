from setuptools import find_packages, setup
import os
from glob import glob   

package_name = 'good_boy_mode'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='genesis',
    maintainer_email='jacquetdaniel19@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'cam_tilt_pan = good_boy_mode.cam_tilt_pan:main',
            'mobile_bot = good_boy_mode.mobile_bot:main'
        ],
    },
)
