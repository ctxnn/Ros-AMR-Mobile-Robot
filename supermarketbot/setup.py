from setuptools import setup
import os
from glob import glob

package_name = 'supermarketbot'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'meshes'),
        [f for f in glob('meshes/**/*', recursive=True) if os.path.isfile(f)]),
        (os.path.join('share', package_name, 'world'), glob('world/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Chirag Taneja',
    maintainer_email='chiragtaneja.work@gmail.com',
    description='Autonomous supermarket assistant robot — ROS 2 simulation for product navigation and customer interaction',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_node = supermarketbot.vision:main'
        ],
    },
)
