#! /usr/bin/env python3
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
import os
import math
from reportlab.lib.pagesizes import inch, letter
from reportlab.platypus import SimpleDocTemplate, Table, TableStyle
from reportlab.lib import colors


params_file = os.environ['PARAMS_FILE']

def table_generator(data,result):
    elements=[]
    params_file = os.environ['PARAMS_FILE']
    specs= os.path.join(
        get_package_share_directory('benchmarking_tool'),
        'config',
        params_file+'.yaml'
       )
    with open(specs, 'r') as file:
        robot_specs = yaml.safe_load(file)
        
    pdf_name=robot_specs['experiment_name']
        
    CPU=[]
    Memory=[]    
    
    for i in range(len(data)-4):
        CPU.append(data[i+2][0])
        Memory.append(data[i+2][1])

    empty_matrix=["", "", "", "", "", "", ""]
    summary='The '+result+' '+'Execution time is '+str( data[len(data)-1][6])+". Average CPU usage is " +str('{0:.2f}'.format(sum(CPU)/len(CPU)))+"%. "+"Max CPU usage is " +str(max(CPU))+"%. \n"+  "Average memory usage is " +str('{0:.2f}'.format(sum(Memory)/len(Memory)))+"%. "+" Max memory usage is " +str(max(Memory))+"%. "   
    empty_matrix[0]=summary

    data.append(empty_matrix)
    #Name of user choice 
    doc=SimpleDocTemplate(pdf_name+".pdf",pagesize=letter)
    t=Table(data, 7*[1.1*inch], (len(data))*[0.5*inch])
    t.setStyle(TableStyle([('INNERGRID',(0,0), (-1,-1), 0.25, colors.black),
                           ('BOX',(0,0), (-1,-1), 0.25, colors.black),
                           ('SPAN',(0,0),(0,1)),
                           ('SPAN',(1,0),(1,1)),
                           ('SPAN',(2,0),(2,1)),
                           ('SPAN',(3,0),(3,1)),
                           ('SPAN',(4,0),(4,1)),
                           ('SPAN',(5,0),(5,1)),
                           ('SPAN',(6,0),(6,1)),
                           ('FONTNAME',(0,0),(6,1),'Helvetica-Bold'),
                           ('SPAN',(0,len(data)-1),(6,len(data)-1)),
                           ('FONTNAME',(0,len(data)-1),(6,len(data)-1),'Helvetica-Bold'),
                           ]))
    elements.append(t)
    doc.build(elements)






