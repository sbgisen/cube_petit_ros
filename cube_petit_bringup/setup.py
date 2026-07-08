#!/usr/bin/env python

# Copyright (c) 2026 SoftBank Corp.
# 
# <<licensetext>>

from setuptools import setup
import glob

package_name = "cube_petit_bringup"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob.glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob.glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="gisen",
    maintainer_email="SBGRP-git@g.softbank.co.jp",
    description="cube_petit bringup package",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "startup_announcer = cube_petit_bringup.startup_announcer:main",
        ],
    },
)
