from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'drone_object_classifier'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'msg'), glob('msg/*.msg')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'classification_msgs'],
    zip_safe=False,
    maintainer='hadi',
    maintainer_email='38956768+0xVR@users.noreply.github.com',
    description='Autonomous drone vision system using ROS and PyTorch for real-time object classification.',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_subscriber_node = drone_object_classifier.image_subscriber_node:main',
            'classification_publisher_node = drone_object_classifier.classification_publisher_node:main',
        ],
    },
)
