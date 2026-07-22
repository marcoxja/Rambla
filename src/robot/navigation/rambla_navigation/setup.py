from setuptools import find_packages, setup

package_name = 'rambla_navigation'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/navigate.launch.py']),
        ('share/' + package_name + '/config',
            ['config/nav2.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jack Demarco',
    maintainer_email='jackdemarco.jd@gmail.com',
    description='Nav2 goal navigation against the M5 localization stack: Nav2 bring-up plus behavior_supervisor, the sole /cmd_vel_raw writer.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'behavior_supervisor = rambla_navigation.behavior_supervisor:main',
            'verify_navigation = rambla_navigation.verify_navigation:main',
        ],
    },
)
