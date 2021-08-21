from setuptools import setup
import os
from glob import glob
package_name = 'custom_models'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'launch'), glob('launch/*')),
        (os.path.join('lib', package_name), glob('scripts/*')),
        (os.path.join('share', package_name,'models/traffic_bar_stand'), glob('models/traffic_bar_stand/*.sdf')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='luqman',
    maintainer_email='noshluk2@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'sdf_spawner_node = custom_models.sdf_spawner:main',

        ],
    },
)
