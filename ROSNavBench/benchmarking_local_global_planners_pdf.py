#! /usr/bin/env python3
from hmac import new
from pydoc import plain
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
from reportlab.platypus import Image as Image_pdf
from reportlab.lib import colors
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.enums import TA_CENTER
from reportlab.graphics.charts.legends import Legend, LineLegend, LineSwatch
from matplotlib import pyplot as plt
from PIL import Image
from numpy import asarray
from ROSNavBench.follow_path import circle_points,square_points 
from ROSNavBench.performace_analysis import performance_analysis
from scipy import ndimage
from scipy.interpolate import interp1d

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
    
    # Get the name of config file of the current experiment
    specs = os.environ['PARAMS_FILE']
    # Open config file and extact data
    with open(specs, 'r') as file:
        robot_specs = yaml.safe_load(file)
        
    pdf_name=robot_specs['experiment_name']
    controller_type=robot_specs['controller_type']
    planner_type=robot_specs['planner_type']
    results_directory=robot_specs['results_directory']
    x = robot_specs['spawn_pose_x']     
    y = robot_specs['spawn_pose_y'] 
    trajectory_type= robot_specs['trajectory_type']
    map_path=robot_specs['map_path']
    map_png_path=robot_specs['map_png_path']
    criteria= robot_specs['criteria']
    weights= robot_specs['weights']
    instances_num= robot_specs['instances_num']
      
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
    distance_to_obstacles=[]
    # nested arrays equal to number of controllers 
    # E.g., x_points=[[x points for the 1st controller],[x points for the 2nd controller],...]
    for i in range(len(controller_type)*len(planner_type)*instances_num):
        data.append([])
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
    for k in range(len(planner_type)):
        for i in range(len(controller_type)):
            for q in range(instances_num):
                round_num=k*len(controller_type)*instances_num+i*instances_num+q
                f=open(os.path.join(get_package_share_directory('ROSNavBench'),
                'raw_data',
                pdf_name+'_'+controller_type[i]+planner_type[k]+str(round_num+1)+'.csv'),'r')
                writer=csv.reader(f,quoting=csv.QUOTE_NONNUMERIC,delimiter=' ')
                for lines in  writer:
                    data[round_num].append(lines[:])
             

  
    # Extarct data from data array and arrange them into different arrays
    for k in range(len(controller_type)*len(planner_type)*instances_num):
        for i in range(len(data[k])-5):    #EDIT FOR ADDING THE PATH
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
    print("CPU  ",CPU) 
    print(len(CPU))
    print("CPU global ",global_CPU)   
    print("x points",x_points)   
    print(len(x_points))  
    print("y points",y_points)  
    print(len(y_points)) 
    print("xy points",xy_points)
    print(len(xy_points))
    print("obstacles",distance_to_obstacles)
    print("time ",time)
    # Convert the nested array into tuples to satisfy the requirment of Label() function of reportlab
    for k in range(len(controller_type)*len(planner_type)*instances_num):
        CPU_data[k]=tuple(CPU_data[k])
        Memory_data[k]=tuple(Memory_data[k])
        xy_points[k]=tuple(xy_points[k])    
 
    # Extracting the result as a string from data array 
    def result(round_num):
        result=''
        for i in range(len(data[round_num][len(data[round_num])-3])):  #EDITNG for addin path
            result+=data[round_num][len(data[round_num])-3][i]   #EDITNG for addin path
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
    
    # PDF title
    d=shapes.Drawing(5,40)
    d.add(String(1,20,pdf_name,fontSize=20)) 
    elements.append(d) 
    # A table summarize the results 
    d=shapes.Drawing(250,40)
    d.add(String(1,20,"Comparsion of controllers",fontSize=15)) 
    elements.append(d)  
     
    analysis_data=[]
    box_plot_data=[[],[],[],[],[],[],[]]
    for i in range(len(planner_type)):

        d=shapes.Drawing(250,40)
        d.add(String(1,20,"-Global planner: "+planner_type[i],fontSize=12,fontName= 'Times-Bold')) 
        elements.append(d)  
        table= [["Controller\ntype","Success\n rate\n (%)","Average \nexecution\ntime (sec)","CPU(%)","","Memory usage(%)","Memory usage (%)","Average\nnumber of\nrecoveries","Average\n path\nlength(m)","Min\nproximity to\n obstacles(m)"]]  
        table.append(["","","","Average","Max","Average","Max","","",""])
        for k in range(len(controller_type)): 
            #a loop for each instance 
            instance_results=[]
            instance_CPU=[]
            instance_Memory=[]
            instance_execution_time=[]
            instance_path_length=[]
            instance_recovery=[]
            instance_obstacles=[]
            for q in range(instances_num):
                round_num=i*len(controller_type)*instances_num+k*instances_num+q
                instance_results.append(result(round_num))
                instance_CPU+=CPU[round_num]
                instance_Memory+=Memory[round_num]
                instance_execution_time.append(data[round_num][len(data[round_num])-4][6])
                instance_path_length.append(round(path_length(round_num),2))
                instance_recovery.append(data[round_num][len(data[round_num])-4][4])
                instance_obstacles.append(min(distance_to_obstacles[round_num]))
            box_plot_data[0].append(planner_type[i]+"\n"+controller_type[k])
            box_plot_data[1].append(instance_Memory)
            box_plot_data[2].append(instance_CPU)
            box_plot_data[3].append(instance_obstacles)
            box_plot_data[4].append(instance_execution_time)
            box_plot_data[5].append(instance_path_length)
            box_plot_data[6].append(i*len(controller_type)+k+1)
            print(instance_results)
            print("CPU")
            print(instance_CPU)
            print("memory")
            print(instance_Memory)
            print("time")
            print(instance_execution_time)
            print("path")
            print(instance_path_length)
            print("recovery")
            print(instance_recovery)
            print("obstacles")
            print(instance_obstacles)
    
            ##
            table_data=[]
            table_data.append(controller_type[k])
            table_data.append(str(round((instance_results.count('succeeded')/len(instance_results))*100,2))) #success rate of this combination
            table_data.append(str('{0:.2f}'.format(sum(instance_execution_time)/len(instance_execution_time))))    #Execution time 
            table_data.append(str('{0:.2f}'.format(sum(instance_CPU)/len(instance_CPU))))  # average CPU
            table_data.append(str(max(instance_CPU)))    #Max CPU
            table_data.append(str('{0:.2f}'.format(sum(instance_Memory)/len(instance_Memory)))) #average Memory
            table_data.append(str(max(instance_Memory)))   #Max memory
            table_data.append(str('{0:.2f}'.format(sum(instance_recovery)/len(instance_recovery))))    #Number of recoveries 
            table_data.append(str('{0:.2f}'.format(sum(instance_path_length)/len(instance_path_length)))) #Path length    
            table_data.append(str(min(instance_obstacles)))   #Proximity to obstacles
            table.append(table_data)
     
        analysis_data.append(table)
        t=Table(table)
        t.setStyle(TableStyle([('INNERGRID',(0,0), (-1,-1), 0.25, colors.black),
                            ('BOX',(0,0), (-1,-1), 0.25, colors.black),
                            ('SPAN',(0,0),(0,1)),
                            ('SPAN',(1,0),(1,1)),
                            ('SPAN',(2,0),(2,1)),
                            ('SPAN',(7,0),(7,1)),
                            ('SPAN',(8,0),(8,1)),
                            ('SPAN',(9,0),(9,1)),
                            ('SPAN',(3,0),(4,0)),
                            ('SPAN',(5,0),(6,0)),
                            ('FONTNAME',(0,0),(9,1),'Times-Bold'),
                            ]))
        elements.append(t)  
    # Performace analysis 
    d=shapes.Drawing(250,70)
    d.add(String(1,40,"Performace analysis",fontSize=15)) 
    elements.append(d)  
    print(analysis_data)
    ranking,planners_success_rate,controllers_success_rate=performance_analysis(criteria,analysis_data,weights,planner_type,controller_type)
    d.add(String(1,20,"Based on the criteria:" +", ".join(criteria),fontName= 'Times-Bold'))
    d=shapes.Drawing(250,20*(len(ranking)+1))
    d.add(String(1,20*(len(ranking)+1),"The score of each controller and planner combinations are:",fontName= 'Times-Bold'))
    row_increament=1
    for i in ranking:
        text=i+"  "+str(round(ranking[i],2))
        d.add(String(1,20*(row_increament),text))
        row_increament+=1
    elements.append(d)      
    d=shapes.Drawing(250,20*(len(planners_success_rate)+1))
    d.add(String(1,20*(len(planners_success_rate)+1),"Planners' success rate are:",fontName= 'Times-Bold'))
    row_increament=1
    for i in range(len(planners_success_rate)):
        d.add(String(1,20*(row_increament),planners_success_rate[i]))
        row_increament+=1
    elements.append(d)  
    d=shapes.Drawing(250,20*(len(controllers_success_rate)+1))
    d.add(String(1,20*(len(controllers_success_rate)+1),"Controllers' success rate are:",fontName= 'Times-Bold'))
    row_increament=1
    for i in range(len(controllers_success_rate)):
        d.add(String(1,20*(row_increament),controllers_success_rate[i]))
        row_increament+=1
    print("--------------",box_plot_data[0])
    elements.append(d)   
    fig = plt.figure(figsize =(10, 12),constrained_layout=True)
    plt.subplot(3,1,1).set_title ("Memory(%)",fontdict={'family':'serif','size':12})
    plt.boxplot(box_plot_data[1])
    plt.xticks(ticks =box_plot_data[6] ,labels = box_plot_data[0], rotation = 'horizontal',fontdict={'family':'serif','size':12})
    plt.subplot(3,1,2).set_title("CPU(%)",fontdict={'family':'serif','size':12})
    plt.boxplot(box_plot_data[2])
    plt.xticks(ticks =box_plot_data[6] ,labels = box_plot_data[0], rotation = 'horizontal',fontdict={'family':'serif','size':12})
    plt.subplot(3,1,3).set_title ("Proximity to obstacles(m)",fontdict={'family':'serif','size':12})
    plt.boxplot(box_plot_data[3])
    plt.xticks(ticks = box_plot_data[6],labels = box_plot_data[0], rotation = 'horizontal',fontdict={'family':'serif','size':12})
    plt.savefig(os.path.join(get_package_share_directory('ROSNavBench'),
         'raw_data','graph_box_plot.png'))  
    elements.append(Image_pdf(os.path.join(get_package_share_directory('ROSNavBench'),
         'raw_data','graph_box_plot.png'),500,600))
    fig = plt.figure(figsize =(10, 8),constrained_layout=True)
    plt.subplot(2,1,1).set_title ("Time (sce)",fontdict={'family':'serif','size':12})
    plt.boxplot(box_plot_data[4])
    plt.xticks(ticks =box_plot_data[6] ,labels = box_plot_data[0], rotation ='horizontal',fontdict={'family':'serif','size':12})
    plt.subplot(2,1,2).set_title ("Path Length (m)",fontdict={'family':'serif','size':12}) 
    plt.boxplot(box_plot_data[5])
    plt.xticks(ticks = box_plot_data[6] ,labels = box_plot_data[0], rotation = 'horizontal',fontdict={'family':'serif','size':12})
    plt.savefig(os.path.join(get_package_share_directory('ROSNavBench'),
         'raw_data','graph_box_plot2.png'))  
    elements.append(Image_pdf(os.path.join(get_package_share_directory('ROSNavBench'),
         'raw_data','graph_box_plot2.png'),500,400)) 
    d=shapes.Drawing(250,55)
    d.add(String(1,20,"Graphs",fontSize=15)) 
    elements.append(d)   
    legend_list=[]
    for j in range(math.ceil(len(controller_type)/6)): 
        v=j*6
        legend = LineLegend()
        legend.alignment = 'right'
        legend.x = 1
        legend.y = 20
        legend.deltax = 60
        legend.dxTextSpace = 10
        legend.columnMaximum = 1
        items = 'red blue yellow pink black aqua bisque blueviolet brown burlywood cadetblue chartreuse chocolate cornflowerblue crimson cyan green darkblue darkcyan darkgoldenrod darkgray coral darkgreen darkkhaki darkmagenta darkolivegreen darkorange darkred darksalmon darkseagreen darkslateblue darkslategray darkturquoise darkviolet deeppink deepskyblue dimgray firebrick forestgreen fuchsia grey greenyellow gold hotpink indianred ivory lavender lime maroon navy olive'.split()
        cnp = [] 
        r=6
        if j==(math.ceil(len(controller_type)/6)-1) and len(controller_type)%6!=0:
            r=len(controller_type)%6
        for i in range(0, r):
            l =  LineSwatch()
            l.strokeColor = getattr(colors, items[i+v])
            
            cnp.append((l, controller_type[i+v]))
        legend.colorNamePairs = cnp
        legend_list.append(legend)
        #d.add(legend, 'legend')
        #elements.append(d) 
    drawing = shapes.Drawing(500, 10)
    elements.append(drawing) 
    #Finding a mean line and the probabilty region
    def interpolate_trajectory(x, y, num_points):
        # Create a cumulative distance array as reference
        distance = np.cumsum(np.sqrt(np.ediff1d(x, to_begin=0)**2 + np.ediff1d(y, to_begin=0)**2))
        print("distance",distance)
        distance = distance / distance[-1]  # Normalize distance to [0,1]

        # Interpolate using the distance as reference
        fx = interp1d(distance, x, kind='linear')
        fy = interp1d(distance, y, kind='linear')

        # New normalized distance values
        new_distances = np.linspace(0, 1, num_points)
        return fx(new_distances), fy(new_distances)

    def mean_line(x_arrays,y_arrays):
        new_x_array=[]
        new_y_array=[]
        arrays_length=[]
        for i in range(len(x_arrays)):
            arrays_length.append(len(x_arrays[i]))
        print(arrays_length)
        points_num=max(arrays_length)
        print(points_num)
        for i in range(len(x_arrays)):
            new_x,new_y=interpolate_trajectory(x_arrays[i], y_arrays[i], points_num)
            new_x_array.append(new_x)
            new_y_array.append(new_y)
    
        x_mean=np.mean(new_x_array,axis=0)
        y_mean=np.mean(new_y_array,axis=0)
        std=np.std(new_y_array,axis=0)
        return x_mean,y_mean,std
    #CPU plot 
    #Oter loop prodcuce new graph
    for i in range(len(planner_type)):
        # start the graph 
        plt.figure(figsize=(10, 4))
        plt.ylim(0,100)
        labels_list=[]
        labels_tag=[]
        #inner loop plot a new line 
        for k in range(len(controller_type)): 
           round_num=i*len(controller_type)*instances_num+k*instances_num
           CPU_array=CPU[round_num:round_num+instances_num]
           time_array=time[round_num:round_num+instances_num]
           print("CPU_array",CPU_array)
           print("time_array",time_array)
           
           #Shaded area or dots of actual lines 
        #    for q in range(instances_num):   
        #         actual_points,=plt.plot(time_array[q],CPU_array[q],color=items[k] ,linestyle='dotted',alpha=0.3)
           
           mean_time,mean_CPU,std=mean_line(time_array,CPU_array)
           lmean,=plt.plot(mean_time,mean_CPU,label=planner_type[i]+" "+controller_type[k],color=items[k])
           lsigma=plt.fill_between(mean_time, mean_CPU-std,mean_CPU+std,color=items[k],  alpha=0.2)
           labels_list.append((lmean,lsigma))
           labels_tag.append(controller_type[k]+" Mean +/- stdev")
           #labels_list.append(actual_points)
           #labels_tag.append("Actual CPU")
        
        plt.legend(labels_list,labels_tag,prop={'family':'serif','size':8})
        plt.xlabel('Time(sec)',fontdict={'family':'serif','size':12})
        plt.ylabel('CPU (%)',fontdict={'family':'serif','size':12})
        #save the graph 
        plt.savefig(os.path.join(get_package_share_directory('ROSNavBench'),
            'raw_data','CPU_plot'+str(i)+'.png')) 
     
        # CPU plot
        drawing = shapes.Drawing(500, 30)
        drawing.add(String(1,20,"-Global planner: "+planner_type[i],fontSize=12,fontName= 'Times-Bold'))
        drawing.add(String(200,10,'CPU usage(%) ', fontSize=12, fillColor=colors.black))
        elements.append(drawing)
        # insert the graph to pdf   
        elements.append(Image_pdf(os.path.join(get_package_share_directory('ROSNavBench'),
        'raw_data','CPU_plot'+str(i)+'.png'),500,200))
    
    # Trajectory plot 
    #Trajectory plot on map image 
    # Open map yaml file and extact data
    #new
    if trajectory_type=="circle":
        r= robot_specs['radius']
        waypoints_array=circle_points(x,y,r)
    elif trajectory_type=="several_waypoints":        
        waypoints_array=robot_specs['waypoints']
    elif trajectory_type=="square":            
        side_length=robot_specs['side_length']
        waypoints_array=square_points(x,y,side_length)
    elif trajectory_type=="one_goal": 
        waypoints_array=[[robot_specs['goal_pose_x'],robot_specs['goal_pose_y']]] 
    x_trajectory=[]
    y_trajectory=[]
    #  plot the waypoints
    for i in range(len(waypoints_array)):
        x_trajectory.append(waypoints_array[i][0])
        y_trajectory.append(waypoints_array[i][1])  
    for i in range(len(planner_type)):
        # start the graph 
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
        print("-----------------x is ",x_length,"-----------y is ",y_length)
        labels_list=[]
        labels_tag=[]
        if y_length<x_length:
            ax.imshow(ndimage.rotate(img,90),cmap=plt.cm.gray,extent=[0.05*x_length+origin[1],origin[1],origin[0],0.05*y_length+origin[0]])
            for k in range(len(controller_type)):  
                round_num=i*len(controller_type)*instances_num+k*instances_num
                x_array=x_points[round_num:round_num+instances_num]
                y_array=y_points[round_num:round_num+instances_num]
                mean_x,mean_y,std=mean_line(y_array,x_array)
                #Shaded area or dots of actual lines 
                #for q in range(instances_num):   
                #    actual_points,=plt.plot(y_array[q],x_array[q],color=items[k] ,linestyle='dotted',alpha=0.3)
                #    plt.scatter(y_array[q][len(y_array[q])-1],x_array[q][len(x_array[q])-1],color=items[k] ,marker='o',alpha=0.9,linewidth=0.2)
                lmean,=plt.plot(mean_x,mean_y,label=planner_type[i]+" "+controller_type[k],color=items[k])
                lsigma=plt.fill_between(mean_x, mean_y-std,mean_y+std,color=items[k],  alpha=0.2)
                labels_list.append((lmean,lsigma))
                labels_tag.append(controller_type[k]+" Mean +/- stdev")
                #labels_list.append(actual_points)
                #labels_tag.append("Actual trajectory")
            plt.xlabel('y-axis (meters)',fontdict={'family':'serif','size':12})
            plt.ylabel('x-axis (meters)',fontdict={'family':'serif','size':12})
            #  Plot the initial pose 
            if trajectory_type=="circle": 
                plt.plot(y,x+r,marker='*',c='yellowgreen',ms=13)
            else:
                plt.plot(y,x,marker='*',c='yellowgreen',ms=13)   
            #  plot the global path planner    
            plt.plot(data[round_num][len(data[round_num])-1],data[round_num][len(data[round_num])-2],c='yellowgreen',ms=13)
            plt.scatter(y_trajectory,x_trajectory,marker='*',c='yellowgreen')
        
        else:
            ax.imshow(img,cmap=plt.cm.gray,extent=[origin[0],0.05*y_length+origin[0],origin[1],0.05*x_length+origin[1]])
            for k in range(len(controller_type)):  
                round_num=i*len(controller_type)*instances_num+k*instances_num
                x_array=x_points[round_num:round_num+instances_num]
                y_array=y_points[round_num:round_num+instances_num]
                mean_x,mean_y,std=mean_line(x_array,y_array)
                #Shaded area or dots of actual lines 
                #for q in range(instances_num):   
                   # actual_points,=plt.plot(x_array[q],y_array[q],color=items[k] ,linestyle='dotted',alpha=0.3)
                   # plt.scatter(x_array[q][len(x_array[q])-1],y_array[q][len(y_array[q])-1],color=items[k],marker='o',alpha=0.9,linewidth=0.2)
                lmean,=plt.plot(mean_x,mean_y,label=planner_type[i]+" "+controller_type[k],color=items[k])
                lsigma=plt.fill_between(mean_x, mean_y-std,mean_y+std,color=items[k],  alpha=0.2)
                labels_list.append((lmean,lsigma))
                labels_tag.append(controller_type[k]+" Mean +/- stdev")
                #labels_list.append(actual_points)
                #labels_tag.append("Actual trajectory")
            plt.xlabel('x-axis (meters)',fontdict={'family':'serif','size':12})
            plt.ylabel('y-axis (meters)',fontdict={'family':'serif','size':12})
            #  Plot the initial pose 
            if trajectory_type=="circle": 
                plt.plot(x+r,y,marker='*',c='yellowgreen',ms=13)
            else:
                plt.plot(x,y,marker='*',c='yellowgreen',ms=13)   
            #  plot the global path planner    
            plt.plot(data[round_num][len(data[round_num])-2],data[round_num][len(data[round_num])-1],c='yellowgreen',ms=13)
            plt.scatter(x_trajectory,y_trajectory,marker='*',c='yellowgreen')

        #inner loop plot a new line 
        plt.legend(labels_list,labels_tag,prop={'family':'serif','size':8})   
        plt.savefig(os.path.join(get_package_share_directory('ROSNavBench'),
            'raw_data','map_plot'+str(i)+'.png'))

        image_ratio=y_length/x_length
        scaling_factor=image_ratio/(1000/600)
        x_length=int(600*scaling_factor)
        y_length=int(1000*scaling_factor)
        drawing = shapes.Drawing(500,60)  
        drawing.add(String(1,50,"-Global planner: "+planner_type[i],fontSize=12,fontName= 'Times-Bold')) 
        drawing.add(String(200,40,'Traveled path ', fontSize=12, fillColor=colors.black))
        #drawing.add(Circle(300,10,3,fillColor=colors.white))   
        #drawing.add(String(308,10,'Final pose ',fontSize=12, fillColor=colors.black))
        star_vertices=[201.99,9.78,205.88,11.194,201.04,11.708,200,16,198.8,11.6,194.1,11.091,198.07,9.46,196.3,5.277,200,8,203.86,5.406,201.99,9.78]
        drawing.add(Polygon(star_vertices, fillColor=colors.yellowgreen,strokeColor=colors.yellowgreen))
        drawing.add(String(210,10,'Initial pose ',fontSize=12, fillColor=colors.black))
        drawing.add(Line(1,13,7,13, strokeColor=colors.yellowgreen, strokeWidth=3))
        drawing.add(String(9,10,'Global planner path ',fontSize=12, fillColor=colors.black))    
        drawing.add(String(115,10,'* ',fontSize=13, fillColor=colors.yellowgreen))
        drawing.add(String(124,10,'Waypoints ',fontSize=12, fillColor=colors.black)) 

        elements.append(drawing) 
        elements.append(Image_pdf(os.path.join(get_package_share_directory('ROSNavBench'),
            'raw_data','map_plot'+str(i)+'.png'),y_length,x_length))
            
    first_failure=0  
    for k in range(len(planner_type)):
        for i in range(len(controller_type)):
            for q in range(instances_num):
                log_msgs=[]
                round_num=k*len(controller_type)*instances_num+i*instances_num+q
                if result(round_num)=="failed" or result(round_num)=='goal has an invalid return status!':
                    if first_failure==0:
                        d=shapes.Drawing(250,40)
                        d.add(String(1,20,"Failure report",fontSize=15)) 
                        elements.append(d)
                        first_failure=1
                
                    #  Opening the csv of the error msgs       
                    f=open(os.path.join(get_package_share_directory('ROSNavBench'),
                    'raw_data',
                    pdf_name+'_'+controller_type[i]+planner_type[k]+"_error_msgs_"+str(round_num+1)+'.csv'),'r')
                    writer=csv.reader(f,quoting=csv.QUOTE_NONNUMERIC,delimiter=' ')
                    for lines in  writer:
                        log_msgs.append(lines[:])  
       
                    table= [["Global planner: "+planner_type[k],'',''],["Controller:  "+controller_type[i]+'    Round#:'+str(round_num+1),'',''],["Logger_name", "Level", "Message"]]  
                    table.append([""])
                    for j in range(len(log_msgs)):
                        table.append(log_msgs[j])

                    t=Table(table)
                    t.setStyle(TableStyle([('INNERGRID',(0,0), (-1,-1), 0.25, colors.black),
                                   ('BOX',(0,0), (-1,-1), 0.25, colors.black),
                                   ('SPAN',(0,2),(0,3)),
                                   ('SPAN',(1,2),(1,3)),
                                   ('SPAN',(2,2),(2,3)),
                                   ('SPAN',(0,0),(2,0)),
                                   ('SPAN',(0,1),(2,1)),
                                   ('FONTNAME',(0,0),(8,1),'Times-Bold'),
                                   ('FONTSIZE',(0,0), (-1,-1),8)
                            ]))
                    elements.append(t) 
                    elements.append(Drawing(500, 10))


    doc.build(elements)
    