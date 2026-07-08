from setuptools import find_packages, setup

package_name = 'rambla_traversal'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/wander.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jack Demarco',
    maintainer_email='jackdemarco.jd@gmail.com',
    description='Autonomous reactive wander behavior for producing a reproducible mapping-run traverse, without requiring a map or navigation stack. Publishes to cmd_vel_raw, always passing through rambla_safety.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'traversal_node = rambla_traversal.traversal_node:main',
        ],
    },
)
