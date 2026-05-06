from setuptools import find_packages, setup

package_name = 'mujoco_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Shahar Cohen',
    maintainer_email='cohenshahar17@gmail.com',
    description='ROS2 bridge between MuJoCo simulation and the VLA task tree',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge_node = mujoco_bridge.bridge_node:main',
        ],
    },
)
