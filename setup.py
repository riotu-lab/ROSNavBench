from setuptools import setup
import os
from glob import glob
from pathlib import Path

package_name = 'ROSNavBench'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share/' + package_name, 'launch/'), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name), glob('rviz/*.rviz')),       
        (os.path.join('share/' + package_name, 'config/'), glob('./config/*')),
        (os.path.join('share/' + package_name, 'script/'), glob('./script/*')),  
        (os.path.join('share/' + package_name, 'results/'), glob('./results/*')),
        (os.path.join('share/' + package_name, 'raw_data/'), glob('./raw_data/*')),     
        (os.path.join('share/' + package_name, 'simulations/models/block'), glob('simulations/models/block/*')),
        # Install turtlebot3_world model files (preserving directory structure)
        (os.path.join('share/' + package_name, 'simulations/models/turtlebot3_world'), 
         [str(p) for p in Path('simulations/models/turtlebot3_world').iterdir() if p.is_file()]),
        (os.path.join('share/' + package_name, 'simulations/models/turtlebot3_world/meshes'), 
         [str(p) for p in Path('simulations/models/turtlebot3_world/meshes').iterdir() if p.is_file()]),
        (os.path.join('share/' + package_name, 'simulations/models/turtlebot3_waffle_gz'), glob('simulations/models/turtlebot3_waffle_gz/*')),
        (os.path.join('share/' + package_name, 'simulations/maps/'), glob('simulations/maps/*')),
        (os.path.join('share/' + package_name, 'simulations/worlds/'), glob('simulations/worlds/*')),
        (os.path.join('share/' + package_name, 'simulations/urdf/'), glob('simulations/urdf/*')),
 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Fatimah-Alahmed',
    maintainer_email='falahmed@psu.edu.sa',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'follow_path = ROSNavBench.follow_path:main',   
        'benchmarking_single_controller =  ROSNavBench.benchmarking_single_controller:main', 
        'marker_publisher = ROSNavBench.marker_publisher:main', 
        'trajectory_generator= ROSNavBench.trajectory_generator:main',
        'reset_robot= ROSNavBench.reset_robot:main',
        'pdf_generator= ROSNavBench.pdf_generator:main'
   
        ],
    },
)
