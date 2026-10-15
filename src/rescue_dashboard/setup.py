from glob import glob

from setuptools import setup

package_name = 'rescue_dashboard'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/web', glob('web/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bhavarth',
    maintainer_email='bhavarthkokane25@gmail.com',
    description='Web mission control for the rescue swarm (http://localhost:8080).',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'dashboard = rescue_dashboard.server:main',
        ],
    },
)
