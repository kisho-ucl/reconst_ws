from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'yolo_tools'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name], 
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name,'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name,'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aichi2204',
    maintainer_email='yuyaa199908@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rgbd2cloud = '+ package_name + '.rgbd2cloud:main',
            'rgbd2mesh = '+ package_name + '.rgbd2mesh:main',
        ],
    },
)
