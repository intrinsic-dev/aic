import os
import subprocess
import sys
import venv

pypi_deps = ["lerobot"]


def setup_venv(venv_dir: str):
    if os.path.exists(venv_dir) and os.path.isdir(venv_dir):
        print(f"Virtual environment already exists at: {os.path.abspath(venv_dir)}")
    else:
        builder = venv.EnvBuilder(
            prompt="aic", with_pip=True, system_site_packages=True
        )
        builder.create(venv_dir)
        print(f"Virtual environment created at: {os.path.abspath(venv_dir)}")

    # Install pip dependencies
    pip_executable = f"{venv_dir}/bin/pip"
    try:
        subprocess.check_call([pip_executable, "install", *pypi_deps])
        print("Succesfully installed pip dependencies in the virtual environment.")
    except subprocess.CalledProcessError as e:
        print(f"Error installing pip dependencies: {e}")
        sys.exit(1)
