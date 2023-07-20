from setuptools import setup
import os
from glob import glob

package_name = 'benchmarking_tool'

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
        (os.path.join('share/' + package_name, 'models/'), glob('./models/wall/*')),
        (os.path.join('share/' + package_name, 'world/'), glob('./world/*')),
        (os.path.join('share/' + package_name, 'map/'), glob('./map/*')),
        (os.path.join('share/' + package_name, 'script/'), glob('./script/*')),        
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
        'send_goal = benchmarking_tool.pose_goal_publisher:main',
        'generate_pdf = benchmarking_tool.pdf_generator:main',      
        'send_pose = benchmarking_tool.send_goal:main', 
        'follow_path = benchmarking_tool.follow_path:main',
        'trry_pdf = benchmarking_tool.trail_pdf:main', 
        'mangment = benchmarking_tool.mangment:main', 
        'listener = benchmarking_tool.listener_bt:main',  
        'pdf_generator =  benchmarking_tool.pdf_generator_v3:main',            
        ],
    },
)
