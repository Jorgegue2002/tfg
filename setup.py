from setuptools import find_packages, setup

#Launch del mundo
import os
from glob import glob

package_name = 'tfg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        #Launch del mundo
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'hospital', 'worlds'), glob('hospital/worlds/*')),


    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jorge-manso-cordoba',
    maintainer_email='jmansocordoba@gmail.com',
    description='Mi TFG sobre un robot que entrega de medicamentos',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
