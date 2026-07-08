import os

from setuptools import find_packages, setup

package_name = 'rambla_control_panel'


def _static_data_files():
    # Mirrors static/ 1:1 under share/<pkg>/static/ so server.py can locate
    # it via get_package_share_directory at runtime (installed, not
    # source-tree, path - matches how config/launch files are found
    # elsewhere in this repo, e.g. rambla_localization's ekf.yaml lookup).
    data_files = []
    static_root = os.path.join(package_name, 'static')
    for dirpath, _dirnames, filenames in os.walk(static_root):
        if not filenames:
            continue
        install_dir = os.path.join('share', package_name, os.path.relpath(dirpath, package_name))
        data_files.append((install_dir, [os.path.join(dirpath, f) for f in filenames]))
    return data_files


setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/control_panel.launch.py']),
    ] + _static_data_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jack Demarco',
    maintainer_email='jackdemarco.jd@gmail.com',
    description='Web-based drive/debug control panel for the simulated or real robot.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'control_panel = rambla_control_panel.control_panel_node:main',
        ],
    },
)
