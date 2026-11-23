from glob import glob
import os
from setuptools import setup

package_name = 'dummy_ros2_description'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='muzixiaowen',
    maintainer_email='xin.li@switchpi.com',
    description='Robot description for the Dummy arm.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={'console_scripts': []},
)
