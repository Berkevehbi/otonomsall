from setuptools import setup

package_name = 'robotaxi_behavior_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='berke',
    maintainer_email='berke@example.com',
    description='Behavior planner node for robotaxi',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'behavior_planner_node = robotaxi_behavior_planner.behavior_planner_node:main'
        ],
    },
)
