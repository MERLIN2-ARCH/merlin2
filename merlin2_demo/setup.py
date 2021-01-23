import os
from glob import glob
from setuptools import setup

package_name = 'merlin2_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*_launch.py')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='miguel',
    maintainer_email='mgons@unileon.es',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'merlin2_demo_node = merlin2_demo.merlin2_demo_node:main',
            'merlin2_hi_navigation_action = merlin2_demo.merlin2_hi_navigation_action:main',
            'merlin2_check_wp_action = merlin2_demo.merlin2_check_wp_action:main',
            'merlin2_mdpi_node = merlin2_demo.merlin2_mdpi_node:main'
        ],
    },
)