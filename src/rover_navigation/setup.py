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
            "depth_bridge_node = rover_navigation.depth_bridge_node:main",
            "dashboard_node = rover_navigation.dashboard_node:main",
            "vo_node = rover_navigation.vo_node:main",
            "stop_and_go_filter_node = rover_navigation.stop_and_go_filter_node:main",
        ],
    },
)
