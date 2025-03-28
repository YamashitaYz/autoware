from setuptools import setup
import os
from glob import glob
package_name = 'autoware_wheeltec_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.xml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='trc',
    maintainer_email='wheeltec@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'autoware_wheeltec_interface = autoware_wheeltec_interface.autoware_wheeltec_interface:main'
        ],
    },
)
