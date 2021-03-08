from setuptools import setup

# Python imports
import os
from glob import glob

package_name = 'turtle_manipulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('./launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='karol',
    maintainer_email='01149434@pw.edu.pl',
    description='Simple package for controlling turtle from turtlesim.',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move = turtle_manipulation.move:main'
        ],
    },
)
