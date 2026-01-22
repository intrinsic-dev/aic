from setuptools import find_packages, setup

package_name = "aic_example_policies"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Morgan Quigley",
    maintainer_email="morganquigley@intrinsic.ai",
    description="Example policies for the AI challenge",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
)
