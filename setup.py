from setuptools import find_packages, setup

package_name = 'dual_drone_1'

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
    maintainer='vimal-pranav',
    maintainer_email='vimal-pranav@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_water_tracker = dual_drone_1.single_drone:main',
            'geotag_publisher = dual_drone_1.dual_drone_cov:main',
            'geotag_subscriber = dual_drone_1.dual_drone_irr:main',
        ],
    },
)
