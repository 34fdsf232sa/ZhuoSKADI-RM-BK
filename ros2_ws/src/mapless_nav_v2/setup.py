from setuptools import find_packages, setup

package_name = 'mapless_nav_v2'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Zhuo RM Team',
    maintainer_email='zhuo@rm.com',
    description='Mapless Navigation V2 — COD-architecture-inspired',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
