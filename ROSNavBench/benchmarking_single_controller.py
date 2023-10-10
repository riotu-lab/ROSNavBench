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
from reportlab.graphics.charts.barcharts import VerticalBarChart
from matplotlib import pyplot as plt
from PIL import Image
from numpy import asarray
from reportlab.platypus import Image as Image_pdf
from ROSNavBench.follow_path import circle_points,square_points 
from ROSNavBench.performace_analysis import performance_analysis_repeatability

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
    # Create a matrix of specifc size
    def create_matrix(cols):
        return ["" for _ in range(cols)]   
    # Get the name of config file of the current experiment
    specs = os.environ['PARAMS_FILE']
    # Open config file and extact data
    # specs= os.path.join(
    #     get_package_share_directory('ROSNavBench'),
    #     'config',
    #     params_file+'.yaml'
    #    )
    with open(specs, 'r') as file:
        robot_specs = yaml.safe_load(file)
        
    pdf_name=robot_specs['experiment_name']
    trails_num = robot_specs['trails_num']
    controller_type=robot_specs['controller_type']
    planner_type=robot_specs['planner_type']
    results_directory=robot_specs['results_directory']
    x = robot_specs['spawn_pose_x']     
    y = robot_specs['spawn_pose_y'] 
    map_path=robot_specs['map_path']
    map_png_path=robot_specs['map_png_path']    
    trajectory_type= robot_specs['trajectory_type']
    trails_num = robot_specs['trails_num']
    if trails_num>0:
       controller_type=[controller_type[0]]*trails_num    
    # list of arrays to hold data of experiment
    # This is a way to arrange data to be used for analysis and ploting
    controller_num=len(controller_type)
    # data=[[]]*controller_num
    # summary=[[]]*controller_num
    # CPU=[[]]*controller_num
    # Memory=[[]]*controller_num    
    # CPU_data=[[]]*controller_num
    # Memory_data=[[]]*controller_num
    # xy_points=[[]]*controller_num
    # x_points=[[]]*controller_num
    # y_points=[[]]*controller_num
    # time=[[]]*controller_num
    # global_CPU=[[]]*controller_num
    # global_Memory=[[]] *controller_num
    # global_x_points=[[]]*controller_num
    # global_y_points=[[]]*controller_num
    # global_time=[[]]*controller_num
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
    distance_to_obstacles=[]  
    global_distance_to_obstacles=[]
    #log_msgs=[]    
    # nested arrays equal to number of controllers 
    # E.g., x_points=[[x points for the 1st controller],[x points for the 2nd controller],...]
    for i in range(len(controller_type)):
        data.append([])
        #log_msgs.append([])        
        CPU.append([])
        Memory.append([])   
        CPU_data.append([])
        Memory_data.append([])
        xy_points.append([])
        x_points.append([])
        y_points.append([])
        time.append([])
        distance_to_obstacles.append([])
    # open each csv file for the different used controllers, and add all data to data array
    for i in range(len(controller_type)):
        f=open(os.path.join(get_package_share_directory('ROSNavBench'),
        'raw_data',
        pdf_name+'_'+controller_type[i]+planner_type[0]+str(i+1)+'.csv'),'r')
        writer=csv.reader(f,quoting=csv.QUOTE_NONNUMERIC,delimiter=' ')
        for lines in  writer:
            data[i].append(lines[:])
        # #  Opening the csv of the error msgs 
        # f=open(os.path.join(get_package_share_directory('ROSNavBench'),
        # 'raw_data',
        # pdf_name+'_'+controller_type[i]+"_error_msgs_"+str(i+1)+'.csv'),'r')
        # writer=csv.reader(f,quoting=csv.QUOTE_NONNUMERIC,delimiter=' ')
        # for lines in  writer:
        #     log_msgs[i].append(lines[:])     


    # Extarct data from data array and arrange them into different arrays
    for k in range(len(controller_type)):
        for i in range(len(data[k])-5):
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
            distance_to_obstacles[k].append(data[k][i+2][7])
            global_distance_to_obstacles.append(data[k][i+2][7])
    print( "CPU",CPU)
    print("Mmemory ",Memory)
    print("Time", time)
    print("saftey", distance_to_obstacles)
    print("memory global",global_Memory)
    # Convert the nested array into tuples to satisfy the requirment of Label() function of reportlab
    for k in range(len(controller_type)):
        CPU_data[k]=tuple(CPU_data[k])
        Memory_data[k]=tuple(Memory_data[k])
        xy_points[k]=tuple(xy_points[k])        
    # Extracting the result as a string from data array 
    def result(controller_num):
        result=''
        for i in range(len(data[controller_num][len(data[controller_num])-3])):
            result+=data[controller_num][len(data[controller_num])-3][i]
        return result
    # Generating the pdf
    elements=[]
    if results_directory!='':
       doc=SimpleDocTemplate(os.path.join(results_directory,
        pdf_name+".pdf"),pagesize=A4)
    else:
       doc=SimpleDocTemplate(os.path.join(get_package_share_directory('ROSNavBench'),
        'results',
        pdf_name+".pdf"),pagesize=A4)        
    d=shapes.Drawing(5,40)
    d.add(String(1,20,pdf_name,fontSize=20)) 
    elements.append(d) 

    d=shapes.Drawing(250,40)
    d.add(String(1,20,"Comparsion of controllers",fontSize=15)) 
    elements.append(d)  
    d=shapes.Drawing(250,40)
    d.add(String(1,20,"-Global planner: "+planner_type[0],fontSize=12,fontName= 'Times-Bold')) 
    elements.append(d)  
    table= [["Controller\ntype","Result","Execution\nTime (sec)","CPU(%)","","Memory usage(%)","Memory usage (%)","Number of\nrecoveries","Path\nlength\n(m)","Proximity\nto\n obstacles(m)"]]  
    table.append(["","","","Average","Max","Average","Max","","",""])
    for k in range(len(controller_type)): 
        table_data=[]
        if trails_num>0:
            table_data.append(controller_type[k]+' #'+str(k+1))
        else:
            table_data.append(controller_type[k])
        table_data.append(result(k))
        table_data.append(str(data[k][len(data[k])-4][6]))    
        table_data.append(str('{0:.2f}'.format(sum(CPU[k])/len(CPU[k]))))
        table_data.append(str(max(CPU[k])))
        table_data.append(str('{0:.2f}'.format(sum(Memory[k])/len(Memory[k]))))
        table_data.append(str(max(Memory[k])))
        table_data.append(str(data[k][len(data[k])-4][4]))
        table_data.append(str(round(path_length(k),2)))
        table_data.append(str(min(distance_to_obstacles[k])))
        table.append(table_data)
     

    t=Table(table)
    t.setStyle(TableStyle([('INNERGRID',(0,0), (-1,-1), 0.25, colors.black),
                            ('BOX',(0,0), (-1,-1), 0.25, colors.black),
                            ('SPAN',(0,0),(0,1)),
                            ('SPAN',(1,0),(1,1)),
                            ('SPAN',(2,0),(2,1)),
                            #('SPAN',(3,0),(3,1)),
                            #('SPAN',(4,0),(4,1)),
                            #('SPAN',(5,0),(5,1)),
                            #('SPAN',(6,0),(6,1)),
                            ('SPAN',(7,0),(7,1)),
                            ('SPAN',(8,0),(8,1)),
                            ('SPAN',(9,0),(9,1)),
                            ('SPAN',(3,0),(4,0)),
                            ('SPAN',(5,0),(6,0)),
                            ('FONTNAME',(0,0),(9,1),'Times-Bold'),
                            #('SPAN',(0,len(data)-1),(6,len(data)-1)),
                            #('FONTNAME',(0,len(data)-1),(6,len(data)-1),'Helvetica-Bold'),
                            ]))
    elements.append(t)    
    # Performace analysis 
    d=shapes.Drawing(250,40)
    d.add(String(1,20,"Performace analysis",fontSize=15)) 
    elements.append(d)  
    data_variation,success_rate,time_11,path_11=performance_analysis_repeatability([table],planner_type,controller_type)
    d=shapes.Drawing(250,20)
    d.add(String(1,20,success_rate))
    #elements.append(d)         
    d=shapes.Drawing(250,20*len(data_variation))
    d.add(String(1,20*len(data_variation),"The range of each criteria is:"))
    for i in range(len(data_variation)):
        d.add(String(1,20*i,data_variation[i]))
    #elements.append(d) 
    d=shapes.Drawing(250,40)
    d.add(String(1,20,"Graphs",fontSize=15)) 
    elements.append(d)   

    for j in range(math.ceil(len(controller_type)/8)): 
        d = Drawing(500, 50)
        v=j*8
        legend = LineLegend()
        legend.alignment = 'right'
        legend.x = 1
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
        #elements.append(d) 
    # Box plot 

    data_1 = global_Memory
    data_2 = global_CPU
    data_3 = global_distance_to_obstacles
    data_4 = time_11
    data_5 =path_11
    #data = [data_1, data_2, data_3, data_4, data_5]
 
    fig = plt.figure(figsize =(10, 6))
    plt.subplots_adjust(wspace= 0.75)
    plt.subplot(1,5,1) 
    plt.boxplot(data_1)
    plt.xticks(ticks = [1] ,labels = ["Memory (%)"], rotation = 'horizontal',fontdict={'family':'serif','size':12})
    plt.subplot(1,5,2) 
    plt.boxplot(data_2)
    plt.xticks(ticks = [1] ,labels = ["CPU (%)"], rotation = 'horizontal',fontdict={'family':'serif','size':12})
    plt.subplot(1,5,3) 
    plt.boxplot(data_3)
    plt.xticks(ticks = [1] ,labels = ["Proximity to\n obstcales (m)"], rotation = 'horizontal',fontdict={'family':'serif','size':12})
    plt.subplot(1,5,4) 
    plt.boxplot(data_4)
    plt.xticks(ticks = [1] ,labels = ["Time (sec)"], rotation = 'horizontal',fontdict={'family':'serif','size':12})
    plt.subplot(1,5,5) 
    plt.boxplot(data_5)
    plt.xticks(ticks = [1] ,labels = ["Path \nLength (m)"], rotation = 'horizontal',fontdict={'family':'serif','size':12})
    plt.savefig(os.path.join(get_package_share_directory('ROSNavBench'),
         'raw_data','graph_box_plot.png'))
    drawing = shapes.Drawing(500,10)  
    drawing.add(String(200,10,'Performace analysis ', fontSize=12, fillColor=colors.black))
    elements.append(drawing) 
    elements.append(Image_pdf(os.path.join(get_package_share_directory('ROSNavBench'),
         'raw_data','graph_box_plot.png'),500,300)) 

    # CPU plot
    ####NEW
   
    legend = LineLegend()
    legend.alignment = 'right'
    legend.x = 1
    legend.y = 270
    legend.deltax = 60
    legend.dxTextSpace = 10
    legend.columnMaximum = 1
    items = 'red blue yellow pink black aqua bisque blueviolet brown burlywood cadetblue chartreuse chocolate cornflowerblue crimson cyan green darkblue darkcyan darkgoldenrod darkgray coral darkgreen darkkhaki darkmagenta darkolivegreen darkorange darkred darksalmon darkseagreen darkslateblue darkslategray darkturquoise darkviolet deeppink deepskyblue dimgray firebrick forestgreen fuchsia grey greenyellow gold hotpink indianred ivory lavender lime maroon navy olive'.split()
    cnp = []
    l =  LineSwatch()
    l.strokeColor = getattr(colors, items[0])
    cnp.append((l, "Max CPU"))
    l =  LineSwatch()
    l.strokeColor = getattr(colors, items[1])
    cnp.append((l, "Average CPU"))        
    legend.colorNamePairs = cnp
        

    drawing = Drawing(500, 310)
    lab=Label()
    lab.setOrigin(0,130)
    lab.angle=90
    lab.setText('CPU(%)')  
    plot_data=[[],[]]
    for i in range(len(controller_type)):
        plot_data[0].append(max(CPU[i])) 
        plot_data[1].append((sum(CPU[i])/len(CPU[i]))) 
    
    plot_data[0]=tuple(plot_data[0])
    plot_data[1]=tuple(plot_data[1])
    print(plot_data)
    catogries=[]
    for i in range(len(controller_type)):
        catogries.append(str(i+1))
    print(catogries)
    bc = VerticalBarChart()
    bc.x = 40
    bc.y = 35
    bc.height = 220
    bc.width = 450
    bc.data = plot_data
    bc.strokeColor = colors.black

    bc.valueAxis.valueMin =0
    bc.valueAxis.valueMax = 100
    print("global",global_CPU)
    bc.valueAxis.configure(global_CPU) 
    bc.groupSpacing=2 
    bc.categoryAxis.labels.boxAnchor = 'ne'
    bc.categoryAxis.labels.dx = 1
    bc.categoryAxis.labels.dy = -2
    bc.categoryAxis.labels.angle = 30
    bc.categoryAxis.categoryNames = catogries
    drawing.add(String(200,300,'CPU usage(%) ', fontSize=12, fillColor=colors.black))
    drawing.add(legend, 'legend')
    drawing.add(lab)
    drawing.add(bc)
    drawing.add(String(200,5,'Trail # ', fontSize=12, fillColor=colors.black))
    elements.append(drawing)    

    # Memory usage plot
    legend = LineLegend()
    legend.alignment = 'right'
    legend.x = 1
    legend.y = 270
    legend.deltax = 60
    legend.dxTextSpace = 10
    legend.columnMaximum = 1
    items = 'red green blue yellow pink black aqua bisque blueviolet brown burlywood cadetblue chartreuse chocolate cornflowerblue crimson cyan darkblue darkcyan darkgoldenrod darkgray coral darkgreen darkkhaki darkmagenta darkolivegreen darkorange darkred darksalmon darkseagreen darkslateblue darkslategray darkturquoise darkviolet deeppink deepskyblue dimgray firebrick forestgreen fuchsia grey greenyellow gold hotpink indianred ivory lavender lime maroon navy olive'.split()
    cnp = []
    l =  LineSwatch()
    l.strokeColor = getattr(colors, items[0])
    cnp.append((l, "Max Memory Usage (%)"))
    l =  LineSwatch()
    l.strokeColor = getattr(colors, items[1])
    cnp.append((l, "Average Memory Usage (%)"))        
    legend.colorNamePairs = cnp

    drawing = Drawing(500, 350)
    lab=Label()
    lab.setOrigin(0,130)
    lab.angle=90
    lab.setText('Memory(%)')
    plot_data=[[],[]]
    for i in range(len(controller_type)):
        plot_data[0].append(max(Memory[i])) 
        plot_data[1].append((sum(Memory[i])/len(Memory[i]))) 
    
    plot_data[0]=tuple(plot_data[0])
    plot_data[1]=tuple(plot_data[1])
    print(plot_data)
    print(catogries)
    bc = VerticalBarChart()
    bc.x = 40
    bc.y = 35
    bc.height = 220
    bc.width = 450
    bc.data = plot_data
    bc.strokeColor = colors.black

    bc.valueAxis.valueMin =0
    bc.valueAxis.valueMax = 100
    #axis_scalling(min(global_Memory),max(global_Memory),0)
    
    bc.valueAxis.configure(global_Memory) 
    bc.groupSpacing=2 
    bc.categoryAxis.labels.boxAnchor = 'ne'
    bc.categoryAxis.labels.dx = 1
    bc.categoryAxis.labels.dy = -2
    bc.categoryAxis.labels.angle = 30
    bc.categoryAxis.categoryNames = catogries
    drawing.add(String(200,300,'Memory usage ', fontSize=12, fillColor=colors.black))
    drawing.add(legend, 'legend')
    drawing.add(lab)
    drawing.add(bc)
    drawing.add(String(200,5,'Trail # ', fontSize=12, fillColor=colors.black))
    elements.append(drawing)   

     
    # Trajectory plot 
    with open(map_path, 'r') as file:
        map_specs = yaml.safe_load(file)
        
    origin=map_specs['origin']
    resolution=map_specs['resolution']
    img=Image.open(map_png_path)
    numpydata=asarray(img)

    x_length=len(numpydata)
    y_length=len(numpydata[0]) # width
    img=plt.imread(map_png_path)
    fig, ax=plt.subplots()
    ax.imshow(img,cmap=plt.cm.gray,extent=[origin[0],0.05*y_length+origin[0],origin[1],0.05*x_length+origin[1]])
    if trajectory_type=="circle":
       r= robot_specs['radius']
       waypoints_array=[[x+r,y]]
       waypoints_array+=circle_points(x,y,r)
       print(waypoints_array)
    elif trajectory_type=="several_waypoints":
        waypoints_array=[[x,y]]
        waypoints_array+=robot_specs['waypoints']
    elif trajectory_type=="square": 
         waypoints_array=[[x,y]]
         side_length=robot_specs['side_length']
         waypoints_array=square_points(x,y,side_length)
    elif trajectory_type=="one_goal": 
        waypoints_array=[[x,y],[robot_specs['goal_pose_x'],robot_specs['goal_pose_y']]]

    x_trajectory=[]
    y_trajectory=[]
    for i in range(len(waypoints_array)):
        x_trajectory.append(waypoints_array[i][0])
        y_trajectory.append(waypoints_array[i][1])           
    plt.scatter(x_trajectory,y_trajectory,marker='*',c='yellowgreen')
    for p in range(len(controller_type)):
        plt.plot(x_points[p],y_points[p],c='grey',linestyle='dashed')
        plt.plot(x_points[p][len(x_points[p])-1],y_points[p][len(y_points[p])-1],c='grey',marker='o', ms=10)  

    plt.xlabel('x-axis (meters)',fontdict={'family':'serif','size':12})
    plt.ylabel('y-axis (meters)',fontdict={'family':'serif','size':12})

    if trajectory_type=="circle": 
        plt.plot(x+r,y,marker='*',c='yellowgreen',ms=13)
    else:
        plt.plot(x,y,marker='*',c='yellowgreen',ms=13)       
    plt.plot(data[0][len(data[0])-2],data[0][len(data[0])-1],c='yellowgreen',ms=13)       
    plt.savefig(os.path.join(get_package_share_directory('ROSNavBench'),
        'raw_data','map_plot.png'))
    image_ratio=y_length/x_length
    scaling_factor=image_ratio/(500/300)
    x_length=int(300*scaling_factor)
    y_length=int(500*scaling_factor)
    drawing = shapes.Drawing(500,60)  
    drawing.add(String(200,40,'Traveled path ', fontSize=12, fillColor=colors.black))
    drawing.add(Circle(300,10,3,fillColor=colors.grey,strokeColor=colors.grey))   
    drawing.add(String(308,10,'Final pose ',fontSize=12, fillColor=colors.black))
    star_vertices=[201.99,9.78,205.88,11.194,201.04,11.708,200,16,198.8,11.6,194.1,11.091,198.07,9.46,196.3,5.277,200,8,203.86,5.406,201.99,9.78]
    drawing.add(Polygon(star_vertices, fillColor=colors.yellowgreen,strokeColor=colors.yellowgreen))
    drawing.add(String(210,10,'Initial pose ',fontSize=12, fillColor=colors.black))
    drawing.add(Line(1,13,7,13, strokeColor=colors.yellowgreen, strokeWidth=3))
    drawing.add(String(9,10,'Global planner path ',fontSize=12, fillColor=colors.black))    
    drawing.add(String(115,10,'x ',fontSize=13, fillColor=colors.yellowgreen))
    drawing.add(String(124,10,'Waypoints ',fontSize=12, fillColor=colors.black))   
    drawing.add(String(370,10,'-- ',fontSize=15, fillColor=colors.grey))
    drawing.add(String(382,10,'Robot path ',fontSize=12, fillColor=colors.black))          
    elements.append(drawing) 
    elements.append(Image_pdf(os.path.join(get_package_share_directory('ROSNavBench'),
        'raw_data','map_plot.png'),y_length,x_length))
    
    
    first_failure=0 
    for i in range(len(controller_type)): 
        log_msgs=[]
        if result(i)=="failed" or result(i)=='goal has an invalid return status!':
            if first_failure==0:
                    d=shapes.Drawing(250,40)
                    d.add(String(1,20,"Failure report",fontSize=15)) 
                    elements.append(d)
                    first_failure=1
            #  Opening the csv of the error msgs       
            f=open(os.path.join(get_package_share_directory('ROSNavBench'),
            'raw_data',
            pdf_name+'_'+controller_type[i]+"_error_msgs_"+str(i+1)+'.csv'),'r')
            writer=csv.reader(f,quoting=csv.QUOTE_NONNUMERIC,delimiter=' ')
            for lines in  writer:
                log_msgs.append(lines[:])              
            d=shapes.Drawing(250,40)
            d.add(String(1,20,"Log messages of "+controller_type[i]+ " #"+str(i),fontSize=15)) 
            elements.append(d)  
            table= [["Logger_name", "Level", "Message"]]  
            table.append([""])
            for k in range(len(log_msgs[i])):
                table.append(log_msgs[i][k])

     

            t=Table(table)
            t.setStyle(TableStyle([('INNERGRID',(0,0), (-1,-1), 0.25, colors.black),
                                   ('BOX',(0,0), (-1,-1), 0.25, colors.black),
                                   ('SPAN',(0,0),(0,1)),
                                   ('SPAN',(1,0),(1,1)),
                                   ('SPAN',(2,0),(2,1)),

                               ('FONTNAME',(0,0),(8,1),'Helvetica-Bold'),
                               ('FONTSIZE',(0,0), (-1,-1),8)
                               #('SPAN',(0,len(data)-1),(6,len(data)-1)),
                               #('FONTNAME',(0,len(data)-1),(6,len(data)-1),'Helvetica-Bold'),
                            ]))
            elements.append(t) 
    doc.build(elements)
    