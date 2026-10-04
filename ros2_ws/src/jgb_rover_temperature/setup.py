from glob import glob

from setuptools import setup

package_name = 'jgb_rover_temperature'

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
    description='MCP9808 driver, simulated sensor and temperature heatmap for jgb_rover.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mcp9808_driver = jgb_rover_temperature.mcp9808_driver:main',
            'sim_mcp9808 = jgb_rover_temperature.sim_mcp9808:main',
            'temperature_mapper = jgb_rover_temperature.temperature_mapper:main',
        ],
    },
)
