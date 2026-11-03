from glob import glob

from setuptools import find_packages, setup

package_name = "biped_gait"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    package_data={"biped_gait": ["data/*.npz"]},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/assets", glob("assets/*.svg")),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="biped",
    maintainer_email="you@example.com",
    description="Quasi-static walking gait for the BipedRobot V2",
    license="MIT",
    entry_points={"console_scripts": ["quasi_static_walk = biped_gait.quasi_static_walk:main", "compile_walk = biped_gait.plan_store:main"]},
)
