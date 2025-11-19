import os

from colcon_core.distutils.commands.symlink_data import symlink_data
from setup_venv import setup_venv
from setuptools import setup
from setuptools.command.build import build
from setuptools.command.develop import develop
from setuptools.command.install import install

package_name = "aic_venv"
venv_dir = "venv"


def venv_target_dir(install_base: str) -> str:
    return os.path.join(install_base, "share", "aic_venv", "venv")


class CustomBuild(build):
    def run(self):
        super().run()
        setup_venv(os.path.join(self.build_base, venv_dir))


class CustomInstall(install):
    def run(self):
        super().run()
        target = venv_target_dir(self.install_base)
        if not os.path.exists(target):
            os.symlink(os.path.join(self.build_base, venv_dir), target)


class CustomDevelop(develop):
    def run(self):
        super().run()
        setup_venv(os.path.join(os.getcwd(), venv_dir))


class CustomSymlinkData(symlink_data):
    def run(self):
        super().run()
        target = venv_target_dir(self.install_dir)
        if not os.path.exists(target):
            os.symlink(os.path.join(os.getcwd(), "venv"), target)


setup(
    name=package_name,
    cmdclass={
        "build": CustomBuild,
        "install": CustomInstall,
        "develop": CustomDevelop,
        "symlink_data": CustomSymlinkData,
    },
    packages=[],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("bin/", ["aic_venv_python3"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="koonpeng",
    maintainer_email="koonpeng@intrinsic.ai",
)
