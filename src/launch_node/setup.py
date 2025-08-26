from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'launch_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=[
        'setuptools',
        'pyserial',
        ],
    zip_safe=True,
    maintainer='jelie',
    maintainer_email='jeremialieks@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joy_to_steer = launch_node.joy_to_steer:main',
            'joy_publisher_dummy = launch_node.joy_publisher_dummy:main'
        ],
    },
)
