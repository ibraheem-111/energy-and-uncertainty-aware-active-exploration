from setuptools import find_packages, setup
from glob import glob

package_name = 'cave_exploration'
share_dir = 'share/' + package_name

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (share_dir, ['package.xml']),
        (share_dir + '/launch', glob('launch/*.py')),
        (share_dir + '/rviz', glob('rviz/*.rviz')),
        (share_dir + '/cave_simple_03', glob('cave_simple_03/*.sdf')),
        (share_dir + '/cave_simple_03', glob('cave_simple_03/*.dot')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ibraheem',
    maintainer_email='ibraheemsal@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gz_to_px4_odom = cave_exploration.gz_to_px4_odom:main',
            'ros_odom_to_px4_odom = cave_exploration.ros_odom_to_px4_odom:main',
            'exploration_offboard = cave_exploration.exploration_offboard:main',
        ],
    },
)
