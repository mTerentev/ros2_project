from setuptools import setup

package_name = 'balancing_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',['launch/' + package_name + '.launch.py']),
        ('share/' + package_name + '/sdf',['sdf/' + package_name + '.sdf'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mike',
    maintainer_email='miterentev22@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "spawn = balancing_robot.spawn:main",
            "regulator = balancing_robot.regulator:main",
            "controller = balancing_robot.controller:main",
            "imu_decoder = balancing_robot.imu:main",
            "teleop_node = balancing_robot.teleop:main"
        ],
    },
)
