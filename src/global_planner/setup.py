from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'global_planner'  

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Config
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # Maps
        *[
            (os.path.join('share', package_name, os.path.relpath(root, '.')),
             [os.path.join(root, file) for file in files])
            for root, dirs, files in os.walk('maps')
        ]
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lukas',
    maintainer_email='lukas.kutsch@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wp_follower_node = global_planner.wp_follower_node:main',
            'goal_publisher = global_planner.goal_publisher:main',
            'centerline_publisher = global_planner.centerline_publisher:main'
        ],
    },
)
