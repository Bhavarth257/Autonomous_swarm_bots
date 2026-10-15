from setuptools import setup

package_name = 'rescue_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bhavarth',
    maintainer_email='bhavarthkokane25@gmail.com',
    description='Hexapod autonomy: traversability, Nav2/SLAM config, frontier exploration.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'traversability = rescue_navigation.traversability:main',
            'explorer = rescue_navigation.explorer:main',
            'teleop = rescue_navigation.wasd_teleop:main',
            'wall_guard = rescue_navigation.wall_guard:main',
            'wasd_teleop = rescue_navigation.wasd_teleop:main',
        ],
    },
)
