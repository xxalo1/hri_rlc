from setuptools import find_packages, setup

package_name = 'rlc_controller'

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
    maintainer='g201951870',
    maintainer_email='asme.kfupm.website@gmail.com',
    description='Kinova Gen3 controller node for the RLC stack.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gen3_controller = rlc_controller.nodes.controller_node:main',
        ],
    },
)
