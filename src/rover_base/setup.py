from setuptools import find_packages, setup

package_name = "rover_base"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Michael Miao",
    maintainer_email="miaom3649@gmail.com",
    description="Hardware abstraction layer for the autonomous rover.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "ultrasonic_sensor_node = rover_base.ultrasonic_sensor_node:main",
            "camera_node = rover_base.camera_node:main",
            "drive_node = rover_base.drive_node:main",
        ],
    },
)
