from glob import glob

from setuptools import find_packages, setup

setup(
    name='biped_balance', version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/biped_balance']),
        ('share/biped_balance', ['package.xml', 'README.md']),
        ('share/biped_balance/config', glob('config/*.yaml')),
        ('share/biped_balance/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True, maintainer='biped', maintainer_email='you@example.com',
    description='Separate IMU balance experiments for the BipedRobot V2', license='MIT',
    entry_points={'console_scripts': ['balance_node = biped_balance.node:main']},
)
