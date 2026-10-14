from glob import glob

from setuptools import find_packages, setup

package_name = "biped_mujoco"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],   # runtime deps (numpy, mujoco) are in package.xml / the README, so a missing one gives a clear error
    zip_safe=True,
    maintainer="biped",
    maintainer_email="you@example.com",
    description="MuJoCo simulation of the BipedRobot V2",
    license="MIT",
    entry_points={
        "console_scripts": [
            "mujoco_biped = biped_mujoco.mujoco_node:main",
            "biped_mujoco_check = biped_mujoco.check:main",
        ],
    },
)
