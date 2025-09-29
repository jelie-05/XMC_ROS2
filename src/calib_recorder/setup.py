from setuptools import setup

package_name = 'calib_recorder'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='RGB and ToF depth synchronized recorder for calibration datasets',
    license='MIT',
    entry_points={
        'console_scripts': [
            'calib_recorder = calib_recorder.calib_recorder:main',
        ],
    },
)
