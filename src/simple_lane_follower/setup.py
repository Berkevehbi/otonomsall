from setuptools import setup

package_name = 'simple_lane_follower'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sen',
    maintainer_email='sen@sen.com',
    description='Minimal Lane Following Node with Perception Model',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_lane_follower_node = simple_lane_follower.simple_lane_follower_node:main'
        ],
    },
)

