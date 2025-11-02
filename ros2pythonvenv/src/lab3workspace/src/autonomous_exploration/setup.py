from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'autonomous_exploration'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Install the package resource index file.
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Install the package.xml file.
        ('share/' + package_name, ['package.xml']),
        # Install all launch files from the launch directory into the share/<package>/launch directory!
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vincenzo',
    maintainer_email='vincenzolombardi99@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
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
