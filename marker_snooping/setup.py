from setuptools import setup
import os

from glob import glob
from setuptools import setup

package_name = 'marker_snooping'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files. This is the most important line here!
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Marco Lapolla',
    maintainer_email='marco.lapolla5gmail.com',
    description='Marker Snooping via PTU Control',
    license='BSD',
    entry_points={
        'console_scripts': [
        'marker_snooping = marker_snooping.snoop_around:main',
        ],
    },
)
