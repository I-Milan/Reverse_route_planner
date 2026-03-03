from setuptools import setup
import os
from glob import glob

package_name = 'robot_navigation_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Ha később launch fájlt is akarsz, itt add hozzá
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'model = robot_navigation_pkg.model:main',
            'reverse_route_planner = robot_navigation_pkg.reverse_route_planner:main',
            'visualization = robot_navigation_pkg.visualization:main',
        ],
    },
)