from setuptools import find_packages, setup

package_name = 'jointstatereader'

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
    maintainer='lycheeai',
    maintainer_email='contact@lycheeai-hub.com',
    description='ROS2 hardware driver for SO100 robot arm - reads joint states from physical robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_state_reader = jointstatereader.joint_state_reader:main',
        ],
    },
)
