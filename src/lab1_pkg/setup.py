from setuptools import find_packages, setup

package_name = 'lab1_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/lab1_launch.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='erickj1',
    maintainer_email='your_email@example.com',
    description='ROS 2 Python package for lab1',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = lab1_pkg.talker:main',
            'relay = lab1_pkg.relay:main',
        ],
    },
)