from setuptools import find_packages, setup

package_name = 'teleop_brazo'

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
    maintainer='franco',
    maintainer_email='franco@todo.todo',
    description='Interactive keyboard teleoperation node for a 4-DOF robotic arm and gripper.',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'teleop_node = teleop_brazo.teleop_node:main'
        ],
    },
)
