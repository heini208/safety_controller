from setuptools import find_packages, setup

package_name = 'safety_controller'

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
    maintainer='marcel',
    maintainer_email='mstemmeler@gmx.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_node = safety_controller.state_machine:main',
            'tree_node = safety_controller.tree:main',
            'robileSim = safety_controller.robileSimObstacleAvoidance:main',
        ],
    },
)
