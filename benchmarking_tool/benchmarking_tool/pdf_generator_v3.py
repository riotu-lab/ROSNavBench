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



##take the three csv , put data into arraies, generate PDF

params_file = os.environ['PARAMS_FILE']

def main():
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
    controller_type=robot_specs['controller_type']

    data=[]
    summary=[]
    CPU=[]
    Memory=[]    
    CPU_data=[]
    Memory_data=[]
    xy_points=[]
    x_points=[]
    y_points=[]
    time=[]
    for i in range(len(controller_type)):
        data.append([])
        CPU.append([])
        Memory.append([])   
        CPU_data.append([])
        Memory_data.append([])
        xy_points.append([])
        x_points.append([])
        y_points.append([])
        time.append([])
    for i in range(len(controller_type)):
        f=open(pdf_name+'_'+controller_type[i]+'.csv','r')
        writer=csv.reader(f,quoting=csv.QUOTE_NONNUMERIC,delimiter=' ')
        for lines in  writer:
            data[i].append(lines[:])


   
        
    print(data)
    global_CPU=[]
    global_Memory=[] 
    global_x_points=[]
    global_y_points=[]
    global_time=[]

    print(data)
    for k in range(len(controller_type)):
        for i in range(len(data[k])-3):
            CPU[k].append(data[k][i+2][0])
            Memory[k].append(data[k][i+2][1])
            time[k].append(data[k][i+2][6])
            x_points[k].append(data[k][i+2][2])
            y_points[k].append(data[k][i+2][3])
            CPU_data[k].append((data[k][i+2][6],data[k][i+2][0]))
            Memory_data[k].append((data[k][i+2][6],data[k][i+2][1]))
            xy_points[k].append((data[k][i+2][2],data[k][i+2][3]))
            global_CPU.append(data[k][i+2][0])
            global_Memory.append(data[k][i+2][1]) 
            global_x_points.append(data[k][i+2][2])
            global_y_points.append(data[k][i+2][3])
            global_time.append(data[k][i+2][6])

    for k in range(len(controller_type)):
        CPU_data[k]=tuple(CPU_data[k])
        Memory_data[k]=tuple(Memory_data[k])
        xy_points[k]=tuple(xy_points[k])        
  
    # print(time)
    # print(y_points)
    # print(y_points)
    # print(xy_points)
    # print(CPU)
    # print(Memory)
    # print(Memory_data)
    # print(CPU_data)
    # print(global_CPU)
    # print(global_Memory)
    # print(global_time)
    # print(global_x_points)
    # print(global_y_points)
     
    def result(controller_num):
        result=''
        for i in range(len(data[controller_num][len(data[controller_num])-1])):
            result+=data[controller_num][len(data[controller_num])-1][i]
        return result
    #empty_matrix=["", "", "", "", "", "", ""]
    #summary='The '+result+' '+'Execution time is '+str( data[len(data)-1][6])+"sec. Average CPU usage is " +str('{0:.2f}'.format(sum(CPU)/len(CPU)))+"%. "+"Max CPU usage is " +str(max(CPU))+"%. \n"+  "Average memory usage is " +str('{0:.2f}'.format(sum(Memory)/len(Memory)))+"%. "+" Max memory usage is " +str(max(Memory))+"%. "   
    #empty_matrix[0]=summary
  
    doc=SimpleDocTemplate(pdf_name+".pdf",pagesize=A4)
    d=shapes.Drawing(5,40)
    d.add(String(5,20,pdf_name,fontSize=20)) 
    elements.append(d) 
    # Calculate the path taken
    def path_length(controller_number):
        path=0
        for j in range(len(xy_points[controller_number])-1):
            path+=math.sqrt(np.square(xy_points[controller_number][j+1][0]-xy_points[controller_number][j][0])+np.square(xy_points[controller_number][j+1][1]-xy_points[controller_number][j][1]))
            
        return path
    for k in range(len(controller_type)): 
        d=shapes.Drawing(500,55)
        d.add(String(5,50,controller_type[k]+" controller",fontSize=11))
        ########################A way to take result
        d.add(String(5,35,'The '+result(k)+' '+'Execution time is '+str(data[k][len(data[k])-2][6])+' sec. '+"Average CPU usage is " +str('{0:.2f}'.format(sum(CPU[k])/len(CPU[k])))+"%. ",fontSize=12))
        d.add(String(5,20,"Max CPU usage is " +str(max(CPU[k]))+"%. "+"Average memory usage is " +str('{0:.2f}'.format(sum(Memory[k])/len(Memory[k])))+"%. "+" Max memory usage is " +str(max(Memory[k]))+"%. ",fontSize=12))   
        d.add(String(5,5,"Number of revoveries is "+str(data[k][len(data[k])-2][4])+". The length of the path taked is "+str(round(path_length(k),2))+' m.',fontSize=12))
        #d.add(String(5,25, 'bt is'+bt, fontSize=10) )       
        elements.append(d)

    #data.append(empty_matrix)
    #Name of user choice 
    


    
    #Graph title 
    # graph_title=ParagraphStyle('title',
    #                            fontName='Helvetica-Bold',
    #                            fontSize=16,
    #                            parent=style['Heading2'],
    #                            alignment=1,
    #                            spaceAfter=14)
    #Title 

    #Paragraph of Summary 
    
    #Plots of all trails sumary ( 3 plots )
    

    
    # Data for plotting graphs 
    # TimeSteps=[]
    # for i in range(round(max(time))):
    #     TimeSteps.append(i)
    #     TimeSteps.append(i+0.5)  
    items = 'red green blue yellow pink black white'.split()    
    def generate_list(min_val,max_val,step_size):

        generated_list=[]
        current_val=min_val
        #if max_val-min_val>step_size:
        while current_val<=max_val:
            generated_list.append(current_val)
            current_val+=step_size  
        return generated_list
        # elif max_val-min_val<step_size:
        #     new_step_size=max_val-min_val
        #     generated_list=[min_val,max_val,max_val+new_step_size,max_val+2
        #                     *new_step_size]
        #     return generated_list
    #CPU of all trails 
    def axis_scalling(min_val,max_val,value):
        difference=max_val-min_val
        if value==0:              #for max value
           if difference<1:
              max_val=max_val+1
           return max_val
        elif value==1:            #for min val 
           if difference<1:
              min_val=min_val-1
           return min_val   
        
    d=shapes.Drawing(5,40)
    d.add(String(5,20,"Graphs",fontSize=15)) 
    elements.append(d)   
    d = Drawing(500, 50)

    legend = LineLegend()
    legend.alignment = 'right'
    legend.x = 5
    legend.y = 40
    legend.deltax = 60
    legend.dxTextSpace = 10
    legend.columnMaximum = 1
    items = 'red green blue yellow pink black white'.split()
    cnp = []
    for i in range(0, len(controller_type)):
        l =  LineSwatch()
        l.strokeColor = getattr(colors, items[i])
        cnp.append((l, controller_type[i]))
    legend.colorNamePairs = cnp
    d.add(legend, 'legend')
    elements.append(d)    
    drawing = shapes.Drawing(500, 310)
    lab=Label()
    lab.setOrigin(0,130)
    lab.angle=90
    lab.setText('CPU(%)')  
    lp = LinePlot()
    lp.x = 40
    lp.y = 35
    lp.height = 220
    lp.width = 450
    lp.data = CPU_data
    lp.joinedLines = 1
    for i in range(len(controller_type)):
        lp.lines[i].strokeColor=getattr(colors, items[i])
    #lp.lines[0].symbol = makeMarker('FilledCircle')
    #lp.lines[1].symbol = makeMarker('Circle')
    #lp.lineLabelFormat = '%2.0f'
    lp.strokeColor = colors.black
    lp.xValueAxis.valueMin = 0
    lp.xValueAxis.valueMax = max(global_time)
    lp.xValueAxis.configure(global_time)
    lp.xValueAxis.labelTextFormat = '%2.1f'
    lp.yValueAxis.valueMin = min(global_CPU)-1
    lp.yValueAxis.valueMax = max(global_CPU)
    lp.yValueAxis.configure(global_CPU) 
    drawing.add(String(200,300,'CPU usage(%) ', fontSize=12, fillColor=colors.black))
    drawing.add(lab)
    drawing.add(lp)
    drawing.add(String(200,5,'Time(sec) ', fontSize=12, fillColor=colors.black))
    elements.append(drawing)
    #Memory usage plot
    drawing = shapes.Drawing(500, 310)
    lab=Label()
    lab.setOrigin(0,130)
    lab.angle=90
    lab.setText('Memory')
    lp = LinePlot()
    lp.x = 40
    lp.y = 35
    lp.height = 220
    lp.width = 450
    lp.data = Memory_data
    lp.joinedLines = 1
    for i in range(len(controller_type)):
        lp.lines[i].strokeColor=getattr(colors, items[i])
    #lp.lines[0].symbol = makeMarker('FilledCircle')
    #lp.lines[1].symbol = makeMarker('Circle')
    #lp.lineLabelFormat 1= '%2.0f'
    lp.strokeColor = colors.black
    lp.xValueAxis.valueMin = 0
    lp.xValueAxis.valueMax = max(global_time)
    lp.xValueAxis.configure(global_time)
    lp.xValueAxis.labelTextFormat = '%2.1f'

    lp.yValueAxis.valueMin = axis_scalling(min(global_Memory),max(global_Memory),1)
    lp.yValueAxis.valueMax = axis_scalling(min(global_Memory),max(global_Memory),0)
    lp.yValueAxis.configure(global_Memory) 

    drawing.add(String(200,300,'Memory usage ', fontSize=12, fillColor=colors.black))
    #drawing.add(200,300
    drawing.add(lab)
    drawing.add(lp)
    # drawing.add(String(200,330,'Memory usage ', fontSize=12, fillColor=colors.black,bottomup=1))
    drawing.add(String(200,5,'Time(sec) ', fontSize=12, fillColor=colors.black))
    elements.append(drawing)
    

   
    # #Trajectory plot 
    drawing = shapes.Drawing(500, 320)
    lab=Label()
    lab.setOrigin(0,130)
    lab.angle=90
    lab.setText('Y-axis')  
    lp = LinePlot()
    lp.x = 40
    lp.y = 35
    lp.height = 220
    lp.width = 450
    lp.data = xy_points
    lp.joinedLines = 1
    for i in range(len(controller_type)):
        lp.lines[i].strokeColor=getattr(colors, items[i])
    #lp.lines[0].symbol = makeMarker('FilledCircle')
    #lp.lines[1].symbol = makeMarker('Circle')
    #lp.lineLabelFormat = '%2.0f'
    lp.strokeColor = colors.black
    lp.xValueAxis.valueMin = axis_scalling(min(global_x_points),max(global_x_points),1)
    lp.xValueAxis.valueMax = axis_scalling(min(global_x_points),max(global_x_points),0)
    lp.xValueAxis.configure(generate_list(min(global_x_points),max(global_x_points),0.5))

    lp.xValueAxis.labelTextFormat = '%2.1f'
    lp.yValueAxis.valueMin = axis_scalling(min(global_y_points),max(global_y_points),1)
    lp.yValueAxis.valueMax = axis_scalling(min(global_y_points),max(global_y_points),0)
    lp.yValueAxis.configure((generate_list(min(global_y_points),max(global_y_points),0.5))) 
    #lp.yValueAxis.valueSteps=generate_list(min(global_y_points),max(global_y_points),0.5)    

    drawing.add(String(200,300,'Traveled path ', fontSize=12, fillColor=colors.black))
    drawing.add(lab)
    drawing.add(lp)
    drawing.add(String(200,5,'x-axis ',fontSize=12, fillColor=colors.black))
    elements.append(drawing)   
    # for k in range(len(controller_type)): 
    #     d=shapes.Drawing(250,40)
    #     d.add(String(200,5,controller_type[k]+" controller",fontSize=11)) 
    #     elements.append([-6.34, -6.25, -6.16, -6.07]d) 
    #     t=Table(data[k], 7*[1.1*inch], (len(data[k]))*[0.5*inch])
    #     t.setStyle(TableStyle([('INNERGRID',(0,0), (-1,-1), 0.25, colors.black),
    #                         ('BOX',(0,0), (-1,-1), 0.25, colors.black),
    #                         ('SPAN',(0,0),(0,1)),
    #                         ('SPAN',(1,0),(1,1)),
    #                         ('SPAN',(2,0),(2,1)),
    #                         ('SPAN',(3,0),(3,1)),
    #                         ('SPAN',(4,0),(4,1)),
    #                         ('SPAN',(5,0),(5,1)),
    #                         ('SPAN',(6,0),(6,1)),
    #                         ('FONTNAME',(0,0),(6,1),'Helvetica-Bold'),
    #                         #('SPAN',(0,len(data)-1),(6,len(data)-1)),
    #                         #('FONTNAME',(0,len(data)-1),(6,len(data)-1),'Helvetica-Bold'),
    #                         ]))
    #     elements.append(t)
    doc.build(elements)
    
