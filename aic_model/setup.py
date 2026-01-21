from setuptools import find_packages, setup

package_name = "aic_model"

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
    description="Control model interface for the AI Challenge",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "aic_model = aic_model.aic_model:main",
        ],
    },
)
