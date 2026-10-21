from setuptools import setup
from glob import glob
from os.path import isfile

setup(
    name="fleet_mapping",
    version="0.1.0",
    packages=["fleet_mapping"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/fleet_mapping"]),
        ("share/fleet_mapping", ["package.xml"]),
    ]
    + [
        ("share/fleet_mapping/" + d, [f for f in glob(d + "/*") if isfile(f)])
        for d in ["config", "launch", "worlds", "web"]
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="bwz",
    maintainer_email="bwz@example.com",
    description="Cooperative TurtleBot fleet",
    license="Apache-2.0",
    entry_points={"console_scripts": ["mapping=fleet_mapping.node:main"]},
)
