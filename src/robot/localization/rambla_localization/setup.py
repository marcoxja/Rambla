from setuptools import find_packages, setup

package_name = 'rambla_localization'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/ekf.launch.py']),
        ('share/' + package_name + '/config', ['config/ekf.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jack Demarco',
    maintainer_email='jackdemarco.jd@gmail.com',
    description='EKF sensor fusion (robot_localization) combining wheel odometry and IMU into a filtered pose/velocity estimate.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'frame_id_fixer = rambla_localization.frame_id_fixer:main',
        ],
    },
)
