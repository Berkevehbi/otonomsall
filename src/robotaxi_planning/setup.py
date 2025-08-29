from setuptools import find_packages, setup

package_name = 'robotaxi_planning'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'filterpy'], # <-- 'filterpy' eklendi
    zip_safe=True,
    maintainer='berke',
    maintainer_email='berkevehbi@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_planner_node = robotaxi_planning.path_planner_node:main',
        ],
    },
)
