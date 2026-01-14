from setuptools import find_packages, setup

package_name = 'spacemouse_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'aic_control_interfaces'],
    zip_safe=True,
    maintainer='Grace Kwak',
    maintainer_email='gkwak@intrinsic.ai',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'spacemouse_teleop = spacemouse_teleop.spacemouse_teleop:main'
        ],
    },
)
