from setuptools import find_packages, setup

#Launch del mundo
import os
from glob import glob

package_name = 'hospital'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        #Directorios que van a salir en la carpeta install
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        # (os.path.join('share', package_name, 'models'), glob('models/*')),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jorge-manso-cordoba',
    maintainer_email='jmansocordoba@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
