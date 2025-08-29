# ~/full_ws/src/robotaxi_perception/setup.py (SADELEŞTİRİLMİŞ NİHAİ HAL)

from setuptools import find_packages, setup

package_name = 'robotaxi_perception'

setup(
    name=package_name,
    version='0.0.0',
    # Bu satır, robotaxi_perception ve lib klasörlerini paket olarak tanır
    packages=find_packages(exclude=['test']),
    # DATAFILES ve PACKAGE_DATA ile ilgili her şeyi sildik.
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='berke',
    maintainer_email='berke@todo.todo',
    description='YOLOP modelini kullanan algılama paketi',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'perception_node = robotaxi_perception.perception_node:main',
            'test_video_publisher = robotaxi_perception.test_video_publisher:main',
            'dynamic_obstacle_tracker_node = robotaxi_perception.dynamic_obstacle_tracker_node:main',
        ],
    },
)