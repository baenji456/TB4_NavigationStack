from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mdn_prediction'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='benjamin.serfling@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dummy_mdn_pred_node = mdn_prediction.dummy_mdn_pred_node:main',
            'mdn_sampler = mdn_prediction.mdn_sampler:main'
        ],
    },
)
