import os
import sys
from glob import glob
from setuptools import setup
from setuptools import find_packages
import setuptools.command.build_py
import distutils.cmd
import distutils.log
import setuptools
import subprocess
from ament_index_python.packages import get_package_share_directory

package_name = 'ikin_l5'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name), glob('urdf/*')),
        (os.path.join('share', package_name), glob('config/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bartek',
    maintainer_email='p.3bartek@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'initpub = ikin_l5.init_pub:main',
            'XYZ_RPY = ikin_l5.XYZ_RPY:main',
            'oint = ikin_l5.oint_control_srv:main',
            'oint_req = ikin_l5.oint_client:main',
            'ocmd = ikin_l5.ocmd:main',
            'ikin = ikin_l5.ikin:main',
            'jint = int_l4.jint_control_srv:main',
            'jint_req = int_l4.jint_client:main'
        ],
    },
)
