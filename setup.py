import os
from glob import glob
from setuptools import setup

package_name = 'oscillating_platform'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # REGISTRA A PASTA LAUNCH:
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # REGISTRA A PASTA RVIZ:
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Carlos Craveiro',
    maintainer_email='carlos.craveiro@usp.br',
    description='Projeto de Pouso em Plataforma Móvel',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # REGISTRE AQUI OS SEUS SCRIPTS PARA O ROS RECONHECER
            'ar_platform_estimator = oscillating_platform.ar_platform_estimator:main',
            'joystick = oscillating_platform.joystick:main',
        ],
    },
)
