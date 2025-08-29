from setuptools import find_packages, setup

package_name = 'robotaxi_hw_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='berke',
    maintainer_email='berkevehbi@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hw_interface_node = robotaxi_hw_interface.hw_interface_node:main',
            'hw_interface_node2 = robotaxi_hw_interface.hw_interface_node2:main',
            'test_vehicle_actuation = robotaxi_hw_interface.test_vehicle_actuation:main',
        ],
    },
)
