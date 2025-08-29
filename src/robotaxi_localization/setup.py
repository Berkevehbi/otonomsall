from setuptools import find_packages, setup

package_name = 'robotaxi_localization'

setup(
    name=package_name,
    version='0.0.1',  # Versiyonu istersen güncelleyebilirsin
    packages=find_packages(
        include=[package_name, package_name + '.*'],
        exclude=['test', 'test.*']
    ),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='berke',
    maintainer_email='berkevehbi@gmail.com',
    description='Robotaxi localization package for ROS 2.',
    license='Apache License 2.0',  # Eğer başka bir lisanssa onu yaz
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'localization_node = robotaxi_localization.localization_node:main',  # Eğer main fonksiyonun varsa!
        ],
    },
)
