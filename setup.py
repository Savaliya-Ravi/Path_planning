from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vx_path_planning'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rav4243s@hs-coburg.de',
    maintainer_email='rav4243s@hs-coburg.de',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'path_planner = vx_path_planning.path_planner:main',
    ],
},
)
