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
        (os.path.join('share/' + package_name, 'model/block/'), glob('./model/block/*')),
        (os.path.join('share/' + package_name, 'model/cabinet/'), glob('./model/cabinet/*')),
        (os.path.join('share/' + package_name, 'model/wall/'), glob('./model/wall/*')),
        (os.path.join('share/' + package_name, 'model/office_L1/meshes/'), glob('./model/office_L1/meshes/*')),
        (os.path.join('share/' + package_name, 'model/office_L1/'), glob('./model/office_L1/*.[sdf]*')),
        (os.path.join('share/' + package_name, 'model/office_L1/'), glob('./model/office_L1/*.[config]*')),
        (os.path.join('share/' + package_name, 'model/Bookshelf/thumbnails/'), glob('./model/Bookshelf/thumbnails/*')),
        (os.path.join('share/' + package_name, 'model/Bookshelf/'), glob('./model/Bookshelf/*.[sdf]*')),
        (os.path.join('share/' + package_name, 'model/Bookshelf/'), glob('./model/Bookshelf/*.[config]*')),
        (os.path.join('share/' + package_name, 'model/Bookshelf/'), glob('./model/Bookshelf/*.[jpg]*')),
        (os.path.join('share/' + package_name, 'model/compact_scenario/'), glob('./model/compact_scenario/*')),
        (os.path.join('share/' + package_name, 'model/cafe_table/'), glob('./model/cafe_table/*.[sdf]*')),
        (os.path.join('share/' + package_name, 'model/cafe_table/'), glob('./model/cafe_table/*.[config]*')),
        (os.path.join('share/' + package_name, 'model/cafe_table/meshes/'), glob('./model/cafe_table/meshes/*')),
        (os.path.join('share/' + package_name, 'model/cafe_table/materials/textures/'), glob('./model/cafe_table/materials/textures/*')),
        (os.path.join('share/' + package_name, 'model/chair/2/'), glob('./model/chair/2/*.[config]*')),
        (os.path.join('share/' + package_name, 'model/chair/2/'), glob('./model/chair/2/*.[sdf]*')),
        (os.path.join('share/' + package_name, 'model/chair/2/meshes/'), glob('./model/chair/2/meshes/*')),
        (os.path.join('share/' + package_name, 'model/chair/2/thumbnails/'), glob('./model/chair/2/thumbnails/*')),
        (os.path.join('share/' + package_name, 'model/chair/'), glob('./model/chair/2/*.[config]*')),
        (os.path.join('share/' + package_name, 'model/chair/'), glob('./model/chair/*.[sdf]*')),
        (os.path.join('share/' + package_name, 'model/chair/meshes/'), glob('./model/chair/meshes/*')),
        (os.path.join('share/' + package_name, 'model/chair/thumbnails/'), glob('./model/chair/thumbnails/*')),        
        (os.path.join('share/' + package_name, 'model/Desk/'), glob('./model/Desk/*.[sdf]*')),
        (os.path.join('share/' + package_name, 'model/Desk/'), glob('./model/Desk/*.[config]*')),
        (os.path.join('share/' + package_name, 'model/Desk/meshes/'), glob('./model/Desk/meshes/*')),
        (os.path.join('share/' + package_name, 'model/Desk/thumbnails/'), glob('./model/Desk/thumbnails/*')),
        (os.path.join('share/' + package_name, 'model/DeskChair/'), glob('./model/DeskChair/*.[sdf]*')),
        (os.path.join('share/' + package_name, 'model/DeskChair/'), glob('./model/DeskChair/*.[config]*')),
        (os.path.join('share/' + package_name, 'model/DeskChair/meshes/'), glob('./model/DeskChair/meshes/*')),
        (os.path.join('share/' + package_name, 'model/DeskChair/thumbnails/'), glob('./model/DeskChair/thumbnails/*')), 
        (os.path.join('share/' + package_name, 'model/DeskChair/materials/textures/'), glob('./model/DeskChair/materials/textures/*')),  
        (os.path.join('share/' + package_name, 'model/OfficeChairGrey/'), glob('./model/OfficeChairGrey/*.[sdf]*')),
        (os.path.join('share/' + package_name, 'model/OfficeChairGrey/'), glob('./model/OfficeChairGrey/*.[config]*')),
        (os.path.join('share/' + package_name, 'model/OfficeChairGrey/meshes/'), glob('./model/OfficeChairGrey/meshes/*')),
        (os.path.join('share/' + package_name, 'model/OfficeChairGrey/thumbnails/'), glob('./model/OfficeChairGrey/thumbnails/*.[png]*')),
        (os.path.join('share/' + package_name, 'model/OfficeChairGrey/thumbnails/meta/'), glob('./model/OfficeChairGrey/thumbnails/meta/*')),            
        




        (os.path.join('share/' + package_name, 'world/'), glob('./world/*')),
        (os.path.join('share/' + package_name, 'map/'), glob('./map/*')),
        (os.path.join('share/' + package_name, 'script/'), glob('./script/*')),  
        (os.path.join('share/' + package_name, 'results/'), glob('./results/*')),
        (os.path.join('share/' + package_name, 'raw_data/'), glob('./raw_data/*')),      
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
        'follow_path = benchmarking_tool.follow_path:main',
        'pdf_generator =  benchmarking_tool.pdf_generator:main',   
        'state_publisher =  benchmarking_tool.state_publisher:main',    
        'try =  benchmarking_tool.trail:main',    
        'benchmarking_single_controller =  benchmarking_tool.benchmarking_single_controller:main',  
        ],
    },
)
