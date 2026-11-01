from glob import glob
from setuptools import find_packages, setup

setup(
    name='biped_cpg', version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/biped_cpg']),
        ('share/biped_cpg', ['package.xml', 'README.md']),
        ('share/biped_cpg/launch', glob('launch/*.py')),
        ('share/biped_cpg/config', glob('config/*.yaml')),
        ('share/biped_cpg/assets', glob('assets/*.gif')),
        ('share/biped_cpg/validation', glob('validation/*')),
    ],
    install_requires=['setuptools', 'numpy'], zip_safe=True,
    maintainer='biped', maintainer_email='you@example.com', license='MIT',
    description='Coupled CPG rhythm with URDF-based kinematics and small body lean',
    entry_points={'console_scripts': ['cpg_walk = biped_cpg.node:main', 'cpg_mujoco = biped_cpg.simulation:main']},
)
