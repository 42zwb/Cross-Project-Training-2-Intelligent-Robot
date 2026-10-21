from setuptools import setup
from glob import glob
from os.path import isfile

setup(
    name="fleet_control",
    version="0.1.0",
    packages=["fleet_control"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/fleet_control"]),
        ("share/fleet_control", ["package.xml"]),
    ]
    + [
        ("share/fleet_control/" + d, [f for f in glob(d + "/*") if isfile(f)])
        for d in ["config", "launch", "worlds", "web"]
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="bwz",
    maintainer_email="bwz@example.com",
    description="Cooperative TurtleBot fleet",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "coordinator=fleet_control.coordinator:main",
            "safety=fleet_control.safety:main",
            "dashboard=fleet_control.dashboard:main",
            "fleetctl=fleet_control.cli:main",
            "evaluate=fleet_control.evaluate:main",
        ]
    },
)
