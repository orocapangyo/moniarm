import os
from glob import glob
from setuptools import find_packages
from setuptools import setup

package_name = 'moniarm_cv'
submodules = "moniarm_cv/submodules"

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
        'Camera node, Find blob by HSV filter'
    ),
    license='Apache License, Version 2.0',
    entry_points={
        'console_scripts': [
            'csi_pub = moniarm_cv.csi_pub:main',
            'usbcam_pub = moniarm_cv.usbcam_pub:main',
            'find_ball = moniarm_cv.find_ball:main',
        ],
    },
)
