from setuptools import find_packages, setup

package_name = 'robotaxi_mission_planner'

setup(
    name=package_name,
    version='0.0.1',  # Versiyonu güncelledim.
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        # 'numpy',      # Eğer kullanıyorsan buraya ekle.
        # 'pandas',     # ...
    ],
    zip_safe=True,
    maintainer='berke',
    maintainer_email='berkevehbi@gmail.com',
    description='Mission planner package for RoboTaxi challenge. Provides path planning and mission sequencing functionalities.',
    license='Apache License 2.0',  # veya kullandığın lisans
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_planner_node = robotaxi_mission_planner.mission_planner_node:main',
            # Diğer scriptler burada eklenebilir:
            # 'path_planner_node = robotaxi_mission_planner.path_planner:main',
        ],
    },
)

