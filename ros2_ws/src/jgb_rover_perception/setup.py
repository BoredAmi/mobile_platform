from glob import glob

from setuptools import setup

package_name = 'jgb_rover_perception'

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
    description='Visual floor scan and ArUco landmarks for jgb_rover.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'visual_floor_scan = jgb_rover_perception.visual_floor_scan:main',
            'aruco_detector = jgb_rover_perception.aruco_detector:main',
        ],
    },
)
