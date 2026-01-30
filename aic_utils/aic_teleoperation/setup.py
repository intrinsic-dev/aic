from setuptools import find_packages, setup

package_name = 'aic_teleoperation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='johntangz',
    maintainer_email='johntangz@intrinsic.ai',
    description='Utility scripts for AIC teleoperation',
    license='Apache-2.0',
    scripts=['scripts/cartesian_keyboard_teleop.py', 'scripts/joint_keyboard_teleop.py'],
    entry_points={
        "console_scripts": [],
    },
)
