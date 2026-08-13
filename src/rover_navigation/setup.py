from setuptools import find_packages, setup

package_name = "rover_navigation"

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
    description="Navigation stack for the autonomous rover.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "dashboard_node = rover_navigation.dashboard_node:main",
            "object_detector_node = rover_navigation.object_detector_node:main",
            "mapping_monitor_node = rover_navigation.mapping_monitor_node:main",
            "object_localizer_node = rover_navigation.object_localizer_node:main",
            "semantic_mapper_node = rover_navigation.semantic_mapper_node:main",
            "semantic_navigation_node = rover_navigation.semantic_navigation_node:main",
        ],
    },
)
