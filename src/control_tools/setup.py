from setuptools import find_packages, setup

package_name = 'control_tools'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='realsense',
    maintainer_email='kisho@ucl.nuee.nagoya-u.ac.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_servo = control_tools.control_servo:main',
            'cmd_rot = control_tools.cmd_rot:main',
            'robot_hand_pos = control_tools.robot_hand_pos:main'
        ],
    },
)
