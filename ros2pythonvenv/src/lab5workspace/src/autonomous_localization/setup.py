from setuptools import find_packages, setup

package_name = 'autonomous_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'autonomous_localization = autonomous_localization.autonomous_localization:main',
            'reset_node = autonomous_localization.reset_node:main',
        ],
    },
)
