from glob import glob

from setuptools import setup

package_name = 'jgb_rover_localization'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ami',
    maintainer_email='azmozgame@gmail.com',
    description='IMU bias calibration and EKF config for jgb_rover.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'imu_bias_calibration = jgb_rover_localization.imu_bias_calibration:main',
        ],
    },
)
