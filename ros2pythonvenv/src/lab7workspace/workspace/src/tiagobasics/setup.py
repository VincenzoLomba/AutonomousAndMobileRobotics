from setuptools import find_packages, setup

package_name = 'tiagobasics'

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
            'simple_images_subscriber_node = nodes_images.simple_images_subscriber_node:main',
            'simple_flippedimages_publisher_node = nodes_images.simple_image_publisher_node:main',
            'simple_points_projector_node = nodes_images.simple_points_projector_node:main',
            'simple_2Dpoints_subscriber_node = nodes_images.simple_2Dpoints_subscriber:main',
            'simple_moveitplanner_posegoal_node = nodes_moveit.simple_moveitplanner_posegoal_node:main'
        ],
    },
)
