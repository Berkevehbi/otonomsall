from setuptools import setup
from glob import glob

package_name = 'yolo_traffic_sign_detector'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        # Model ve config dosyalarını pakete dahil et!
        ('share/' + package_name + '/model', [
            'yolo_traffic_sign_detector/model/my_model.pt',
            'yolo_traffic_sign_detector/model/my_model.onnx'
        ]),        
        ('share/' + package_name + '/data', ['yolo_traffic_sign_detector/data/data.yaml']),
        # Opsiyonel: launch dosyaları
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Senin Adın',
    maintainer_email='senin@emailin.com',
    description='YOLOv8 tabanlı trafik levhası ve işaret tanıma ROS 2 paketi',
    entry_points={
        'console_scripts': [
            'yolo_traffic_sign_detector = yolo_traffic_sign_detector.yolo_traffic_sign_detector:main',
        ],
    },
)
