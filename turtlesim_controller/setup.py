import os
from glob import glob

from setuptools import setup

package_name = 'turtlesim_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='luizcarloscf',
    maintainer_email='luiz.cosmi@edu.ufes.br',
    description='Design a simple controller for the turtle simulation.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller=turtlesim_controller.controller:main',
            'client=turtlesim_controller.controller_client:main'
        ],
    },
)
