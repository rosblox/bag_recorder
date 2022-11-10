from setuptools import setup

package_name = 'bag_recorder'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='max',
    maintainer_email='polzin.max@gmail.com',
    description='Record a ROS bag',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
                'bag_recorder = bag_recorder.bag_recorder:main',
        ],
    },
)
