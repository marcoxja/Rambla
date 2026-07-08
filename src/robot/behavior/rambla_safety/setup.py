from setuptools import find_packages, setup

package_name = 'rambla_safety'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/safety.launch.py']),
        ('share/' + package_name + '/config', ['config/safety.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jack Demarco',
    maintainer_email='jackdemarco.jd@gmail.com',
    description='Local safety-reflex layer: halts/overrides cmd_vel on imminent collision using LiDAR and bumper contact, independent of any map or server.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'safety_node = rambla_safety.safety_node:main',
        ],
    },
)
