from setuptools import find_packages, setup

package_name = 'Exa_Prac'

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
    maintainer_email='sadabarriosrocha@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'robot_sensor = Exa_Prac.robot_sensor_publisher:main',
            'robot_monitor = Exa_Prac.robot_monitor:main',
            'robot_state_node = Exa_Prac.robot_state_node:main',
            'robot_mode_server = Exa_Prac.robot_mode_server:main'
        ],
    },
)
