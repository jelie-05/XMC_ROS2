from setuptools import setup

package_name = 'psoc6_motion_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],  # add pyserial here too
    zip_safe=True,
    maintainer='yunjinli',
    maintainer_email='yunjin.li0817@gmail.com',
    description='Bridge for streaming BMI270 IMU data from the PSoC™ 6 AI Evaluation Kit to ROS 2.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'psoc6_motion_bridge = psoc6_motion_bridge.psoc6_motion_bridge:main',
        ],
    },
)
