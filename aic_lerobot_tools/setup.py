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
    maintainer_email="teokoonpeng@gmail.com",
    entry_points={
        "console_scripts": [],
    },
)
