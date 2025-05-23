from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'mis_nodos'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        #Nodos
        (os.path.join('share', package_name, 'nodos'), glob('nodos/*')),
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
            # Se define el nombre que se va usar de cada nodo
            'entregador = create3_nav.entregador:main',
        ],
    },
)
