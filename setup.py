import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'gp7_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
    (os.path.join('share', package_name, 'meshes/gp7/visual'), glob('meshes/gp7/visual/*')),
    (os.path.join('share', package_name, 'meshes/gp7/collision'), glob('meshes/gp7/collision/*')),
    (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ravindu-rashmika',
    maintainer_email='ravindukrashmika@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
		'robot_controller = gp7_sim.robot_controller:main',
        ],
    },
)
