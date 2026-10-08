from glob import glob

from setuptools import find_packages, setup

package_name = "biped_hardware"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "NOTICE.md", "LICENSE"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools", "pyserial", "pyyaml", "numpy"],
    zip_safe=True,
    maintainer="biped",
    maintainer_email="you@example.com",
    description="Real-robot interface (ST3215 servos, BNO055 IMU, calibration) for the BipedRobot V2",
    license="MIT",
    entry_points={
        "console_scripts": [
            "servo_bridge = biped_hardware.servo_bridge_node:main",
            "imu_node = biped_hardware.imu_node:main",
            "biped_scan = biped_hardware.tools.scan:main",
            "biped_set_id = biped_hardware.tools.set_id:main",
            "biped_calibrate = biped_hardware.tools.calibrate:main",
            "biped_jog = biped_hardware.tools.jog:main",
        ],
    },
)
