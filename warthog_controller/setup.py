from setuptools import find_packages, setup

package_name = 'warthog_controller'

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
    maintainer='ubuntu',
    maintainer_email='willmer@wohlen.se',
    description='Warthog controller package containing p_controller and mission_control nodes',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mission_control_node = warthog_controller.mission_control:main',
            'p_controller_node = warthog_controller.p_controller:main',
            'slow_controller_node = warthog_controller.p_controller:main',
            'visual_servo_node = warthog_controller.visual_servo:main',
        ],
    },
)
