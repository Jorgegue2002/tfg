from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'mis_nodos'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name]),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('mis_nodos/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jorge-manso-cordoba',
    maintainer_email='jmansocordoba@gmail.com',
    description='Nodos personalizados',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'entregador = mis_nodos.entregador:main',
            'initial_pose_publisher = mis_nodos.initial_pose_publisher:main',
            'mux_out_to_cmd_vel = mis_nodos.mux_out_to_cmd_vel:main',
        ],
    },
)
