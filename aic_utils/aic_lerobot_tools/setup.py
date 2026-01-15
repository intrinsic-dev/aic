#
#  Copyright (C) 2026 Intrinsic Innovation LLC
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
#

import os

from setuptools import find_packages, setup

package_name = "aic_lerobot_tools"


def dir_files(path: str) -> list[str]:
    # Discover files based on path relative to this setup.py file.
    src_path = os.path.join(os.path.dirname(__file__), path)
    return [
        os.path.join(path, f)
        for f in os.listdir(path)
        if os.path.isfile(os.path.join(src_path, f))
    ]


setup(
    name=package_name,
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (f"share/{package_name}/contracts", dir_files("contracts")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="koonpeng",
    maintainer_email="koonpeng@intrinsic.ai",
    entry_points={
        "console_scripts": [],
    },
)
