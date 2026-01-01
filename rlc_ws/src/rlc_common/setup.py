from setuptools import find_packages, setup

package_name = 'rlc_common'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    install_requires=[
        'setuptools',
        'pyyaml',
        'control_msgs',
        'rlc_interfaces',
        'std_srvs',
    ],
    zip_safe=True,
    maintainer='g201951870',
    maintainer_email='asme.kfupm.website@gmail.com',
    description='Shared constants and topic/service names for the RLC ROS 2 stack.',
    license='MIT',
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
