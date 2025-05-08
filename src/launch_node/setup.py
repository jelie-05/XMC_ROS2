from setuptools import find_packages, setup

package_name = 'launch_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'pyserial',
        ],
    zip_safe=True,
    maintainer='swadiryus',
    maintainer_email='swadiryus@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joy_to_steer = launch_node.joy_to_steer:main',
        ],
    },
)
