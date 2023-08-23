#! /usr/bin/env python3
import numpy as np
import yaml
import csv 
import os
import math
from ament_index_python.packages import get_package_share_directory
from reportlab.lib.pagesizes import inch, letter, A4
from reportlab.graphics import shapes
from reportlab.graphics.shapes import *
from reportlab.graphics.charts.axes import XCategoryAxis, YCategoryAxis
from reportlab.graphics.charts.lineplots import LinePlot
from reportlab.graphics.charts.textlabels import Label
from reportlab.graphics.widgets.markers import makeMarker
from reportlab.platypus import SimpleDocTemplate, Table, TableStyle
from reportlab.lib import colors
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.enums import TA_CENTER
from reportlab.graphics.charts.legends import Legend, LineLegend, LineSwatch

def main():
    params_file = os.environ['PARAMS_FILE']
    specs= os.path.join(
        get_package_share_directory('benchmarking_tool'),
        'config',
        params_file+'.yaml'
       )
    with open(specs, 'r') as file:
        robot_specs = yaml.safe_load(file)
        
    pdf_name=robot_specs['experiment_name']
    controller_type=robot_specs['controller_type']
    results_directory=robot_specs['results_directory']
    x = robot_specs['spawn_pose_x']     
    y = robot_specs['spawn_pose_y'] 
    trajectory_type= robot_specs['trajectory_type']
    trails_num = robot_specs['trails_num']
    if trails_num>0:
       controller_type=[controller_type[0]]*trails_num 
    elements=[]
    if results_directory!='':
       doc=SimpleDocTemplate(os.path.join(results_directory,
        "TRy"+".pdf"),pagesize=A4)
       
       
    for j in range(math.ceil(len(controller_type)/8)): 
        d = Drawing(500, 50)
        v=j*8
        legend = LineLegend()
        legend.alignment = 'right'
        legend.x = 5
        legend.y = 40
        legend.deltax = 60
        legend.dxTextSpace = 10
        legend.columnMaximum = 1
        items = 'red green blue yellow pink black aqua bisque blueviolet brown burlywood cadetblue chartreuse chocolate cornflowerblue crimson cyan darkblue darkcyan darkgoldenrod darkgray coral darkgreen darkkhaki darkmagenta darkolivegreen darkorange darkred darksalmon darkseagreen darkslateblue darkslategray darkturquoise darkviolet deeppink deepskyblue dimgray firebrick forestgreen fuchsia grey greenyellow gold hotpink indianred ivory lavender lime maroon navy olive'.split()
        cnp = []
        r=8
        if j==(math.ceil(len(controller_type)/8)-1) and len(controller_type)%8!=0:
            r=len(controller_type)%8
        for i in range(0, r):
            l =  LineSwatch()
            l.strokeColor = getattr(colors, items[i+v])
            if trails_num>0:
                cnp.append((l, controller_type[i+v]+' #'+str(i+1+v)))
            else:
                cnp.append((l, controller_type[i+v]))
        legend.colorNamePairs = cnp
        d.add(legend, 'legend')
        elements.append(d)
    
    doc.build(elements)    