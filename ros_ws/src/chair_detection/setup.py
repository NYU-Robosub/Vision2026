from setuptools import setup

package_name = 'chair_detection'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'requirements.txt', 'README.md']),
        ('share/' + package_name + '/launch', ['launch/chair_detection.launch.py']),
        ('share/' + package_name + '/config', ['config/chair_detection.yaml']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='NYU RoboSub',
    maintainer_email='maintainers@example.com',
    description='Live CUDA chair counts from an existing RGB image topic',
    license='Apache-2.0',
    entry_points={'console_scripts': ['chair_detection = chair_detection.node:main']},
)
