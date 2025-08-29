from setuptools import setup
package_name = 'line_detector_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Line detection node using OpenCV',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'line_detector_node = line_detector_pkg.line_detector_node:main',
        ],
    },
)
