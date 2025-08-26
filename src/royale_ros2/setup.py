from setuptools import find_packages, setup

package_name = 'royale_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/royale_ros2']),
        ('share/royale_ros2', ['package.xml']),
        ('share/royale_ros2/launch', ['launch/royale.launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zukimo',
    maintainer_email='jeremialieks@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'royale_node = royale_ros2.royale_node:main',
        ],
    },
)
