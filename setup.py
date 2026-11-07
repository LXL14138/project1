from glob import glob

from setuptools import find_packages, setup

package_name = 'q1_navigation'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README_zh.md', 'VALIDATION.txt']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='ESE4450 project team',
    maintainer_email='maintainer@example.com',
    description='Geometric and kinodynamic RRT navigation using clipped LiDAR',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'q1_navigation = q1_navigation.q1_ros_node:main',
        'q2_navigation = q1_navigation.q2_ros_node:main',
    ]},
)
