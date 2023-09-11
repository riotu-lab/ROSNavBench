from setuptools import setup
import os
from glob import glob

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
        (os.path.join('share/' + package_name, 'simulations/models/'), glob('./models/*')),      
 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tehnanmanna',
    maintainer_email='stehnan@psu.edu.sa',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'follow_path = ROSNavBench.follow_path:main',
        'pdf_generator = ROSNavBench.pdf_generator:main',    
        'benchmarking_single_controller =  ROSNavBench.benchmarking_single_controller:main', 
        'marker_publisher = ROSNavBench.marker_publisher:main', 
        'main_pdf_generator= ROSNavBench.benchmarking_local_global_planners_pdf:main',
        
        
        ],
    },
)
