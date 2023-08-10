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
    '''
    The function performs the following:
    1. Take the data form csv files of the exeriment, and arrange data into arrays
    2. Perform analysis of data such as fining the average and max of CPU
    3. Gnerating plots 
    4. Generating a report in form of pdf
    '''
    # Functions used inside main() function
        
    def path_length(controller_number):
        '''
        Calculate the path taken through adding the distance between each two consective 
        The euclidean distance formula is used.
        
        Args:
           controller_number: Specify which controller by giving the place of the controller in the list
           E.g., xy_points=[[x points for the 1st controller],[x points for the 2nd controller],...]
        
        Returns:
           A path length in decimal form
        '''
        path=0
        for j in range(len(xy_points[controller_number])-1):
            path+=math.sqrt(np.square(xy_points[controller_number][j+1][0]-xy_points[controller_number][j][0])+np.square(xy_points[controller_number][j+1][1]-xy_points[controller_number][j][1]))
            
        return path
    
    def generate_list(min_val,max_val,step_size):
        '''
        Generating a list of values between max and min value with a specfic increament

        Args:
           min_value: minimum value in the list
           max_value: maximum value in the list
           step_size: increment for each element in the list
        Returns:
           A list 
        '''
        generated_list=[]
        current_val=min_val
        while current_val<=max_val:
            generated_list.append(current_val)
            current_val+=step_size  
        return generated_list   

    #CPU of all trails 
    def axis_scalling(min_val,max_val,value):
        '''
        Generate a min or a max value for scalling the axis in the plot.
        It alters the min or max value by 1 if the difference between the max and min value is minor.
        As a result, the plot will be well poltted and not represents the minor varition in CPU or trajectoy poses.
        
        Args:
           min_val: minimum value of the plotted axis
           max_val: maximum value of the plotted axis
           value: 1 for finding the min value, 0 for finding the max value
        
        Returns:
            min_value or max_value
        '''
        difference=max_val-min_val
        if value==0:              #for max value
           if difference<1:
              max_val=max_val+1
           return max_val
        elif value==1:            #for min val 
           if difference<1:
              min_val=min_val-1
           return min_val 
        
    # Get the name of config file of the current experiment
    params_file = os.environ['PARAMS_FILE']
    # Open config file and extact data
    specs= os.path.join(
        get_package_share_directory('benchmarking_tool'),
        'config',
        params_file+'.yaml'
       )
    with open(specs, 'r') as file:
        robot_specs = yaml.safe_load(file)
        
    pdf_name=robot_specs['experiment_name']
    controller_type=robot_specs['controller_type']
    # list of arrays to hold data of experiment
    # This is a way to arrange data to be used for analysis and ploting
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
    global_CPU=[]
    global_Memory=[] 
    global_x_points=[]
    global_y_points=[]
    global_time=[]
    # nested arrays equal to number of controllers 
    # E.g., x_points=[[x points for the 1st controller],[x points for the 2nd controller],...]
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
    # open each csv file for the different used controllers, and add all data to data array
    for i in range(len(controller_type)):
        f=open(os.path.join(get_package_share_directory('benchmarking_tool'),
        'raw_data',
        pdf_name+'_'+controller_type[i]+'.csv'),'r')
        writer=csv.reader(f,quoting=csv.QUOTE_NONNUMERIC,delimiter=' ')
        for lines in  writer:
            data[i].append(lines[:])

    # Extarct data from data array and arrange them into different arrays
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
    # Convert the nested array into tuples to satisfy the requirment of Label() function of reportlab
    for k in range(len(controller_type)):
        CPU_data[k]=tuple(CPU_data[k])
        Memory_data[k]=tuple(Memory_data[k])
        xy_points[k]=tuple(xy_points[k])        
    # Extracting the result as a string from data array 
    def result(controller_num):
        result=''
        for i in range(len(data[controller_num][len(data[controller_num])-1])):
            result+=data[controller_num][len(data[controller_num])-1][i]
        return result
    # Generating the pdf
    elements=[]
    doc=SimpleDocTemplate(os.path.join(get_package_share_directory('benchmarking_tool'),
        'results',
        pdf_name+".pdf"),pagesize=A4)
    d=shapes.Drawing(5,40)
    d.add(String(5,20,pdf_name,fontSize=20)) 
    elements.append(d) 
    # This loop prints a summary for each controller in the experiment
    for k in range(len(controller_type)): 
        d=shapes.Drawing(500,60)
        d.add(String(5,50,controller_type[k]+" controller",fontSize=11))
        d.add(String(5,35,'The '+result(k)+' '+'Execution time is '+str(data[k][len(data[k])-2][6])+' sec. '+"Average CPU usage is " +str('{0:.2f}'.format(sum(CPU[k])/len(CPU[k])))+"%. ",fontSize=12))
        d.add(String(5,20,"Max CPU usage is " +str(max(CPU[k]))+"%. "+"Average memory usage is " +str('{0:.2f}'.format(sum(Memory[k])/len(Memory[k])))+"%. "+" Max memory usage is " +str(max(Memory[k]))+"%. ",fontSize=12))   
        d.add(String(5,5,"Number of revoveries is "+str(data[k][len(data[k])-2][4])+". The length of the path taked is "+str(round(path_length(k),2))+' m.',fontSize=12))      
        elements.append(d)
   
    # Addinf the legnd
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
    # CPU plot
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

    lp.strokeColor = colors.black
    lp.xValueAxis.valueMin = 0
    lp.xValueAxis.valueMax = max(global_time)
    lp.xValueAxis.configure(global_time)
    lp.xValueAxis.labelTextFormat = '%2.1f'
    lp.yValueAxis.valueMin = min(global_CPU)-1
    lp.yValueAxis.valueMax=max(global_CPU)      
    lp.yValueAxis.configure(global_CPU) 
    drawing.add(String(200,300,'CPU usage(%) ', fontSize=12, fillColor=colors.black))
    drawing.add(lab)
    drawing.add(lp)
    drawing.add(String(200,5,'Time(sec) ', fontSize=12, fillColor=colors.black))
    elements.append(drawing)

    # Memory usage plot
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
    lp.strokeColor = colors.black
    lp.xValueAxis.valueMin = 0
    lp.xValueAxis.valueMax = max(global_time)
    lp.xValueAxis.configure(global_time)
    lp.xValueAxis.labelTextFormat = '%2.1f'
    lp.yValueAxis.valueMin = axis_scalling(min(global_Memory),max(global_Memory),1)
    lp.yValueAxis.valueMax = axis_scalling(min(global_Memory),max(global_Memory),0)
    lp.yValueAxis.configure(global_Memory) 
    drawing.add(String(200,300,'Memory usage ', fontSize=12, fillColor=colors.black))
    drawing.add(lab)
    drawing.add(lp)
    drawing.add(String(200,5,'Time(sec) ', fontSize=12, fillColor=colors.black))
    elements.append(drawing)
     
    # Trajectory plot 
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
    lp.strokeColor = colors.black
    lp.xValueAxis.valueMin = axis_scalling(min(global_x_points),max(global_x_points),1)
    lp.xValueAxis.valueMax = axis_scalling(min(global_x_points),max(global_x_points),0)
    lp.xValueAxis.configure(generate_list(min(global_x_points),max(global_x_points),0.5))

    lp.xValueAxis.labelTextFormat = '%2.1f'
    lp.yValueAxis.valueMin = axis_scalling(min(global_y_points),max(global_y_points),1)
    lp.yValueAxis.valueMax = axis_scalling(min(global_y_points),max(global_y_points),0)
    lp.yValueAxis.configure((generate_list(min(global_y_points),max(global_y_points),0.5)))     

    drawing.add(String(200,300,'Traveled path ', fontSize=12, fillColor=colors.black))
    drawing.add(lab)
    drawing.add(lp)
    drawing.add(String(200,5,'x-axis ',fontSize=12, fillColor=colors.black))
    elements.append(drawing)   

    doc.build(elements)
    
