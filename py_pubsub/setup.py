import os
from glob import glob
from setuptools import setup

package_name = 'py_pubsub'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # include launch files
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        # (os.path.join('share', package_name), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jose Laruta',
    maintainer_email='eduardo.laruta@gmail.com',
    description='Basic examples of publisher and subscriber',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = py_pubsub.publisher_member_function:main',
            'listener = py_pubsub.subscriber_member_function:main',
            'simple = py_pubsub.simple_diff_drive:main',
            'odom = py_pubsub.odom_diff_drive:main',
        ],
    },
)
