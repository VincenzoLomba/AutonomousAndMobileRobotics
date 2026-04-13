
from setuptools import find_packages, setup
import os

package_name = 'tiago_exam_tasks'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Installing Task1 launcher:
        (os.path.join('share', package_name, 'launch'), ['launch/task1.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vincenzo',
    maintainer_email='vincenzolombardi99@gmail.com',
    description='A simple package contaning all the developed material in solving the exam project in its three tasks',
    license='No license... be careful when using this material, my fellow Padawan (also note that 42 is the answer to everything)',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
