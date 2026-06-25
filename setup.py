from setuptools import find_packages, setup

package_name = 'realsense_sim_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch',
            ['launch/realsense_sim_driver.launch.py']),
        ('share/' + package_name + '/config',
            ['config/config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='15071194757@163.com',
    description='Align Isaac Sim RGB/depth streams and publish RGB, depth, and PointCloud2.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'aligned_realsense_node = realsense_sim_driver.aligned_realsense_node:main',
        ],
    },
)
