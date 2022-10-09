from setuptools import setup

package_name = 'turtlesim_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'turtlesim_controller=turtlesim_controller.controller:main',
            'turtlesim_go_to=turtlesim_controller.go_to:main'
        ],
    },
)
