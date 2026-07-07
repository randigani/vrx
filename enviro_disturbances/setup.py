from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'enviro_disturbances'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*launch.py'))
        # ('share/' + package_name + '/enviro_disturbances', glob('enviro_disturbances/*.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tech',
    maintainer_email='someone@example.com',
    description='TODO: Package description',
    license='MIT',

    entry_points={
        'console_scripts': [
            'oscillate_current = enviro_disturbances.oscillate_current:main',
            'constant_current = enviro_disturbances.constant_current:main',
            'modulate_current = enviro_disturbances.modulate_current:main'
        ],
    },
)
