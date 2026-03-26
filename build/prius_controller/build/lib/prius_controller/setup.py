from setuptools import find_packages, setup

package_name = 'prius_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mayyad',
    maintainer_email='mayyad@todo.todo',
    description='Prius controller package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller = prius_controller.controller:main',
	     'waypoint_controller = prius_controller.waypoint_controller:main',
        ],
    },
)
