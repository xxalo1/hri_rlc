from setuptools import find_packages, setup

package_name = 'rlc_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'numpy',
    ],
    zip_safe=True,
    maintainer='rwolv',
    maintainer_email='rashed@example.com',
    description='Trajectory planning node for the RLC ROS 2 stack.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'planner_node = rlc_planner.planner_node:main'
        ],
    },
)
