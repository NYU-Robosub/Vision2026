from setuptools import setup

package_name = 'vision_status'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='NYU RoboSub',
    maintainer_email='maintainers@example.com',
    description='Runtime status monitoring for ZED and RTAB-Map streams',
    license='Apache-2.0',
    entry_points={'console_scripts': ['vision_status = vision_status.node:main']},
)
