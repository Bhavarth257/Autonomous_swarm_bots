from setuptools import setup

package_name = 'rescue_perception'

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
    description='Victim detection: YOLOv8 + thermal + lidar placement.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'victim_detector = rescue_perception.detector:main',
        ],
    },
)
