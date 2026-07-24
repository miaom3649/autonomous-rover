from setuptools import find_packages, setup

package_name = "rover_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (
            f"share/{package_name}/launch",
            [
                "launch/teleop.launch.py",
                "launch/nav.launch.py",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Michael Miao",
    maintainer_email="miaom3649@gmail.com",
    description="Launch files for the autonomous rover.",
    license="MIT",
    entry_points={"console_scripts": []},
)
