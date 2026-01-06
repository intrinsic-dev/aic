from setuptools import find_packages, setup

package_name = "lerobot_robot_aic"


setup(
    name=package_name,
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="koonpeng",
    maintainer_email="koonpeng@intrinsic.ai",
    entry_points={
        "console_scripts": [],
    },
)
