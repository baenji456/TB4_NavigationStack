from setuptools import setup

package_name = 'yolo_detection'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/yolo_detection.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='ROS2 node that subscribes to a camera image topic',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_detection_node = yolo_detection.yolo_detection_node:main',
        ],
        'console_scripts': [
            'RotateUntilPersonDetected = yolo_detection.yolo_detection.RotateUntilPersonDetected:main',  # add main() in your node
        ],
        'behavior_tree_nodes': [
            'RotateUntilPersonDetected = yolo_detection.yolo_detection.RotateUntilPersonDetected',
        ],
    },
)
