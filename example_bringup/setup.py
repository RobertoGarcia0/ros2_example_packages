from setuptools import find_packages, setup
from glob import glob 
package_name = 'example_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*')),   # Incluye todos los archivos de launch
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robousr',
    maintainer_email='roberto.gar1748@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "rrr_trajectory_test = example_bringup.rrr_trajectory_test:main",
            "rrr_position_test = example_bringup.rrr_position_test:main",
            "scara_trajectory_test = example_bringup.scara_trajectory_test:main",
            "scara_position_test = example_bringup.scara_position_test:main",
        ],
    },
)
