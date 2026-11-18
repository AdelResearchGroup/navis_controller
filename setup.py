from glob import glob

from setuptools import find_packages, setup

package_name = "navis_controller"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    extras_require={"test": ["pytest"]},
    zip_safe=True,
    maintainer="Daniel Ruan",
    maintainer_email="daniel.ruan@princeton.edu",
    description="ROS 2 package for interfacing with Bunker Pro NAVIS",
    license="Apache License 2.0",
    entry_points={
        "console_scripts": [
            "navis_websocket = navis_controller.navis_websocket:main",
            "navis_http = navis_controller.navis_http:main"
        ],
    },
)
