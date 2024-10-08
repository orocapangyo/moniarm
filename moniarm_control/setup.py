import os
from glob import glob
from setuptools import find_packages
from setuptools import setup

package_name = 'moniarm_control'
submodules = "moniarm_control/submodules"

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=[]),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'param'), glob('param/*.yaml')),
    ],
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    author='ChangWhan Lee',
    author_email='zeta0707@gmail.com',
    maintainer='ChangWhan Lee',
    maintainer_email='zeta0707@gmail.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'Robot arm control nodes'
    ),
    license='Apache License, Version 2.0',
    entry_points={
        'console_scripts': [
            'motor_chase= moniarm_control.motor_chase:main',
            'chase_ball = moniarm_control.chase_ball:main',
            'chase_yolo = moniarm_control.chase_yolo:main',
            'chase_moveit = moniarm_control.chase_moveit:main',
            'mimic_teleop = moniarm_control.mimic_teleop:main',
        ],
    },
)
