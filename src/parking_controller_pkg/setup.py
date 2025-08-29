from setuptools import setup

package_name = 'parking_controller_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/parking_system_launch.py']),
        ('share/' + package_name + '/config', ['config/params.yaml']),
    ],

    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Autonomous parking controller node',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'parking_controller_node = parking_controller_pkg.parking_controller_node:main',
            'path_planner_node = parking_controller_pkg.path_planner:main', # BU SATIRI EKLE
],
    },
)
