from setuptools import find_packages, setup
from glob import glob
package_name = 'example_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*')),   # Incluye todos los archivos de launch
        ('share/' + package_name + '/urdf', glob('urdf/*.urdf')),          # Incluye todos los archivos de urdf
        ('share/' + package_name + '/urdf', glob('urdf/*.stl')),           # Incluye todos los archivos de stl
        ('share/' + package_name + '/urdf/sensors', glob('urdf/sensors/*')),          # Incluye todos los archivos de urdf
        ('share/' + package_name + '/rviz', glob('rviz/*')),          # Incluye todos los archivos de rviz
        ('share/' + package_name + '/config', glob('config/*')),          # Incluye todos los archivos de config
        ('share/' + package_name + '/worlds', glob('worlds/*.world')),          # Incluye todos los archivos de mundos de gazebo
        ('share/' + package_name + '/worlds/models', glob('worlds/models/**/*')),          # Incluye todos los archivos de modelos de gazebo
        
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
        ],
    },
)
