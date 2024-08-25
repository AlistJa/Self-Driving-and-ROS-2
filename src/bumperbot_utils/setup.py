from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'bumperbot_utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.[pxy][yma]*'))),
        (os.path.join('lib', package_name), glob(os.path.join(package_name, '*.[pxy][yma]*'))),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jt',
    maintainer_email='janne.tarsa@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bumperbot_tracker = bumperbot_utils.bumperbot_tracker:main'
        ],
    },
)
