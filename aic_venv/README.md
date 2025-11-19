> [!IMPORTANT]
> Note that this package should never be published.

This is a dummy package to workaround ros's lack of support for pypi packages.

This package defines a colcon hook to source a venv at `colcon_ws/venv`. To create the venv, run

```bash
python3 -m venv --prompt aic --system-site-packages venv
```

After activating the venv, you can install pypi only packages with pip3.

e.g.
```bash
pip3 install lerobot
```

TODO(koonpeng): Should this package include a custom build script that creates the venv and install all packages required?
