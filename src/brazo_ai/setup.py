import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'brazo_ai'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='franco',
    maintainer_email='francomilan007@gmail.com',
    description='Capa de IA segura: percepcion a base_link, scene_state, planificador LLM, executor y safety guard para el brazo de 4 GDL',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera_to_base_node = brazo_ai.camera_to_base_node:main',
            'scene_state_node = brazo_ai.scene_state_node:main',
            'llm_agent_node = brazo_ai.llm_agent_node:main',
            'task_executor_node = brazo_ai.task_executor_node:main',
            'safety_guard_node = brazo_ai.safety_guard_node:main',
            'manual_plan_node = brazo_ai.manual_plan_node:main',
            'calibrate_kabsch = brazo_ai.calibrate_kabsch:main',
            'tag_calibrator = brazo_ai.tag_calibrator:main',
            'red_to_base_printer = brazo_ai.red_to_base_printer:main',
            'experimental_logger = brazo_ai.experimental_logger_node:main',
        ],
    },
)
