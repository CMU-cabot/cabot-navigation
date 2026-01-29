from setuptools import setup
import os
from glob import glob

package_name = 'cabot_sngnn'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Developer',
    maintainer_email='developer@example.com',
    description='SNGNN2D-v2 integration for CaBot',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sngnn_node = cabot_sngnn.sngnn_node:main',
        ],
    },
)
