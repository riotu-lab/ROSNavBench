import io
import pandas as pd
import os 
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
import seaborn as sns 
from PIL import Image
from ROSNavBench.performace_analysis import performance_analysis
from scipy import ndimage
from scipy.interpolate import interp1d
import warnings


def combine_images(buf1, buf2):
    # Load images from BytesIO objects
    img1 = Image.open(buf1)
    img2 = Image.open(buf2)

    # Calculate dimensions for the combined image
    width = img1.width + img2.width
    height = max(img1.height, img2.height)

    # Create a new image with the appropriate size
    combined_img = Image.new('RGB', (width, height))

    # Paste the images onto the combined image
    combined_img.paste(img1, (0, 0))  # Paste the first image at the left
    combined_img.paste(img2, (img1.width, 0))  # Paste the second image to the right of the first

    # You can either save this image or return another BytesIO object
    combined_buf = io.BytesIO()
    combined_img.save(combined_buf, format='PNG')
    combined_buf.seek(0)
    return combined_buf
def combine_row_images(image_row):
    # Start with the first image in the row
    combined_row_image = image_row[0]

    # Combine it with each subsequent image in the row
    for buf in image_row[1:]:
        combined_row_image = combine_images(combined_row_image, buf)

    return combined_row_image


def combine_matrix_images(image_matrix, images_per_row):
    combined_images = []
    image_counts = []  # This list will hold the count of images combined in each row
    image_index = 0

    # Calculate the number of rows based on the images_per_row
    num_rows = len(image_matrix) // images_per_row + (1 if len(image_matrix) % images_per_row else 0)

    # Create row_layout_matrix based on images_per_row
    row_layout_matrix = [images_per_row] * num_rows

    for row_layout in row_layout_matrix:
        current_row_images = []

        for _ in range(row_layout):
            if image_index < len(image_matrix):
                current_row_images.append(image_matrix[image_index])
                image_index += 1

        if current_row_images:
            combined_row = combine_row_images(current_row_images)
            combined_images.append(combined_row)
            image_counts.append(len(current_row_images))  # Add the count for this row

    return combined_images, image_counts
# Example usage
# Assuming image_matrix is your list of images and row_layout_matrix is the layout matrix
# combined_images = combine_matrix_images(image_matrix, row_layout_matrix)


def save_plot_as_png(fig, title,dpi=300):
    """
    Save a matplotlib figure as PNG in a BytesIO object and return it.
    """
    buf = io.BytesIO()
    fig.savefig(buf, format='png', bbox_inches='tight',dpi=dpi)
    buf.seek(0)
    return buf, f'{title}.png'
def calculate_path_deviation(df):
    # Calculate deviation
    df['deviation'] = np.sqrt((df['x_pose'] - df['x_path_plan'])**2 + (df['y_pose'] - df['y_path_plan'])**2)
    print("deviation",df['deviation'].head())
    # Aggregate the deviations by planner and controller
    deviation_sum = df.groupby(['Planner', 'Controller', 'Experiment_ID', 'Iteration_ID','Trajectory_Type'])['deviation'].sum().reset_index()
    return deviation_sum
def plot_path_deviation_heatmap(df,width,height):
    """
    Generate a BytesIO object containing a heatmap showing the path deviation for combinations of controllers and planners.

    Parameters:
    - df: DataFrame containing the data.
    """
    
    deviation_sum=calculate_path_deviation(df)
    deviation_mean = deviation_sum.groupby(['Planner', 'Controller'])['deviation'].mean().unstack()
    print("The meann of dviation",deviation_mean)
    # Plot heatmap
    plt.figure(figsize=(width,height))
    sns.heatmap(deviation_mean, annot=True, cmap='viridis', fmt=".2f")
    plt.title('Path Deviation Heatmap by\n Controller and Planner')
    plt.xlabel('Controller')
    plt.ylabel('Planner')

    # Save the plot as PNG in a BytesIO object
    fig = plt.gcf()
    buf, filename = save_plot_as_png(fig, 'Path_Deviation_Heatmap')
    plt.close()

    return buf, filename
def calculate_path_length(data):
    # Sorting data to ensure correct calculation of path length
    data_sorted = data.sort_values(by=['Experiment_ID', 'Iteration_ID'])

    # Calculate the distance between successive positions
    data_sorted['distance'] = np.sqrt(np.diff(data_sorted['x_pose'], prepend=np.NaN)**2 + 
                                      np.diff(data_sorted['y_pose'], prepend=np.NaN)**2)

    # Sum the distances for each trail (Experiment_ID and Iteration_ID)
    path_length_per_trail = data_sorted.groupby(['Experiment_ID', 'Iteration_ID'])['distance'].sum().reset_index()

    # Merge the path lengths with the original data
    merged_data = path_length_per_trail.merge(data[['Experiment_ID', 'Iteration_ID', 'Planner', 'Controller','Trajectory_Type']], 
                                              on=['Experiment_ID', 'Iteration_ID'])

    # Dropping duplicates to ensure each trail appears only once
    unique_merged_data = merged_data.drop_duplicates(subset=['Experiment_ID', 'Iteration_ID'])

    return unique_merged_data

def calculate_complete_path_length(data):
    # Function to calculate the sum of distances within a group
    def sum_of_distances(group):
        distances = np.sqrt(np.diff(group['x_pose'])**2 + np.diff(group['y_pose'])**2)
        return pd.Series({'distance': distances.sum()})

    # Grouping data by Experiment_ID and Iteration_ID and applying the sum_of_distances function
    path_length_per_trail = data.groupby(['Experiment_ID', 'Iteration_ID', 'Planner', 'Controller', 'Trajectory_Type']).apply(sum_of_distances).reset_index()

    return path_length_per_trail


def create_path_length_barchart(data,width,height):
    """
    Create a bar chart for mean path length of each planner-controller combination.

    Parameters:
    - data: DataFrame containing the dataset.
    """
    # Calculate path length for each trail
    #path_length = calculate_path_length(data)
    path_length = calculate_complete_path_length(data)
    # Calculate the mean path length for each Planner and Controller combination
    mean_path_length = path_length.groupby(['Planner', 'Controller'])['distance'].mean().reset_index()
    # Create bar chart
    plt.figure(figsize=(width,height))
    sns.barplot(x='Planner', y='distance', hue='Controller', data=mean_path_length)
    plt.title('Mean Path Length by\n Planner and Controller')
    plt.xlabel('Planner')
    plt.ylabel('Mean Path Length')
    fig = plt.gcf()

    # Save the plot as PNG
    buf, filename = save_plot_as_png(fig, 'Mean_Path_Length_Planner_Controller')
    plt.close()

    return buf, filename
def create_and_save_bar_chart(df, metric,width,height):
    """
    Create bar charts for the given metric and save them as PNG images.
    """

    # Average metric per planner-controller combination
    #plt.figure(figsize=(6.5, 2))
    plt.figure(figsize=(width,height))
    avg_metrics_combination = df.groupby(['Planner', 'Controller'])[metric].mean().reset_index()
    if metric=="Navigation_time" or metric=="number_of_recoveries":
       filtered_results=df[(df['result']!="In progress")]
       avg_metrics_combination = filtered_results.groupby(['Planner', 'Controller'])[metric].mean().reset_index() 
    sns.barplot(x='Planner', y=metric, hue='Controller', data=avg_metrics_combination)
    plt.title(f'Average {metric} per\n  Planner-Controller Combination')
    plt.ylabel(f'Average {metric}')
    plt.xlabel('Planner')
    plt.xticks(rotation=0)
    plt.legend(title='Controller', loc='upper right')
    fig2 = plt.gcf()
    buf2, filename2 = save_plot_as_png(fig2, f'Average_{metric}_per_Combination')
    plt.close()

    return (buf2, filename2)

def create_and_save_heatmap(df, metric,width,height):
    """
    Create a heatmap for the given metric showing the relationship between planners and controllers.
    """
    # Preparing data for the heatmap
    heatmap_data = df.groupby(['Planner', 'Controller'])[metric].mean().unstack()
    if metric=="Navigation_time" or metric=="number_of_recoveries":
       filtered_results=df[(df['result']!="In progress")]
       heatmap_data = filtered_results.groupby(['Planner', 'Controller'])[metric].mean().unstack()
    # Creating the heatmap
    #plt.figure(figsize=(6.5, 2))
    plt.figure(figsize=(width,height))
    sns.heatmap(heatmap_data, annot=True, fmt=".2f", cmap="YlGnBu")
    plt.title(f'Heatmap of Average {metric} \n for Planners and Controllers')
    plt.ylabel('Planner')
    plt.xlabel('Controller')

    # Saving the heatmap as a PNG image
    fig = plt.gcf()
    buf, filename = save_plot_as_png(fig, f'Heatmap_{metric}_Planners_Controllers')
    plt.close()

    return buf, filename


def plot_metric_distribution_complex_boxplot(df, metric):
    """
    Create and return a complex box plot showing the distribution of a given metric,
    organized by planner, controller, and trajectory type. The width of the plot is further reduced for A4 size.
    """
 

    # Calculate the aspect ratio for each subplot based on the number of unique planners
    n_planners = len(df['Planner'].unique())
   
    # Calculate the width and height for the FacetGrid based on A4 paper size
    # Adjust these values based on your preferences
    a4_width_inches = 8.0
    a4_height_inches = 11.0
    facetgrid_width = a4_width_inches / 2  # Assuming 2 subplots per row
    facetgrid_height = a4_height_inches / 4  # Assuming 4 rows for the number of unique planners
    # Create the FacetGrid
    trajectories=df['Trajectory_Type'].unique()
    trajectories_string = ', '.join(trajectories)

    print("unique traj are ",trajectories)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", category=FutureWarning)
        g = sns.FacetGrid(df, col='Planner', col_wrap=2, height=facetgrid_height, aspect=facetgrid_width/facetgrid_height)
        g.map(sns.boxplot, 'Controller', metric, 'Trajectory_Type', hue_order=trajectories, order=None)
        g.add_legend()
        g.fig.subplots_adjust(top=0.8)
        g.fig.suptitle(f'Complex Distribution of {metric}\n by Planner, Controller, and Trajectory Type')

  

    # Save the plot as PNG
    buf, filename = save_plot_as_png(g.fig, f'Complex_Boxplot_{metric}_Distribution',dpi=300)
    plt.close()
    plot_height=math.ceil(n_planners/2)*facetgrid_height
    return buf, filename,plot_height,a4_width_inches
def calculate_plot_size(num_planners, num_controllers, max_width_pts=480, max_height_pts=700, base_width_in=4, base_height_in=3,base_font_size=12,title_font_size=16):
    """
    Calculate an appropriate plot size based on the number of planners and controllers.

    Parameters:
    - num_planners (int): Number of planners.
    - num_controllers (int): Number of controllers.
    - max_width_pts (int): Maximum plot width in points.
    - max_height_pts (int): Maximum plot height in points.
    - base_width_in (float): Base width of the plot in inches.
    - base_height_in (float): Base height of the plot in inches.

    Returns:
    - (float, float): Tuple containing the width and height of the plot in inches.
    """
    # Conversion factor from points to inches (1 inch = 72 points)
    pts_to_inches = 1 / 72

    # Calculate the maximum width and height in inches
    max_width_in = max_width_pts * pts_to_inches
    max_height_in = max_height_pts * pts_to_inches

    # Calculate scaling factors
    width_scale = (num_controllers / 4)  # example scaling factor, adjust as needed
    height_scale = (num_planners / 4)  # example scaling factor, adjust as needed

    # Calculate plot size
    plot_width = min(base_width_in * width_scale, max_width_in)
    plot_height = min(base_height_in * height_scale, max_height_in)

    # Calculate font sizes
    label_font_size = base_font_size * min(width_scale, height_scale)
    title_font_size = title_font_size

    return plot_width, plot_height,label_font_size,title_font_size


def resize_image_to_max_width(buf, max_width_points):
    """
    Resize the image in the BytesIO object to have a width of max_width_points or less,
    while preserving the aspect ratio. Returns the resized image and its dimensions in points.
    """
    # Load the image from the BytesIO object
    with Image.open(buf) as img:
        max_width = max_width_points* (96 / 72)
       
        if float(img.size[0]) != max_width:
            scaling_factor = max_width / img.size[0]
            width_length = int(float(img.size[0]) * scaling_factor)
            height_length = int(float(img.size[1])* scaling_factor)    
        # Convert width from points to pixels (1 point = 1/72 inch, and assuming 96 pixels/inch)
        max_width_pixels = max_width_points * (96 / 72)

        # Resize the image
        img_resized = img.resize(( width_length, height_length), Image.ANTIALIAS)

        # Save the resized image to a new BytesIO object
        buf_resized = io.BytesIO()
        img_resized.save(buf_resized, format='PNG')
        buf_resized.seek(0)

        # Calculate the final dimensions in points
        final_width_points = width_length / (96 / 72)
        final_height_points = height_length / (96 / 72)

        return buf_resized, (final_width_points, final_height_points)

# Usage example
# max_width_points = 500
# resized_buf, (final_width, final_height) = resize_image_to_max_width(buf, max_width_points)
# Now resized_buf contains the resized image, and final_width, final_height are its dimensions in points

def plot_actual_vs_planned_paths(df,planner, controller):
    """
    Generate and return BytesIO objects for each trajectory type's plot,
    showing the planned path in one color and the actual in another.

    Parameters:
    - df: DataFrame containing the data.
    """
    plots = []
    
    filtered_df = df[(df['Planner'] == planner)& (df['Controller'] == controller)]
    unique_trajectories = filtered_df['Trajectory_Type'].unique()
    for trajectory in unique_trajectories:
        subset = filtered_df[filtered_df['Trajectory_Type'] == trajectory].reset_index()
        print()
        
        plt.figure(figsize=(6.5, 4))
        sns.scatterplot(x=subset['x_pose'], y=subset['y_pose'],data=subset, hue='Iteration_ID')
        sns.scatterplot(x=subset['x_path_plan'], y=subset['y_path_plan'],data=subset,  hue='Iteration_ID')
        
        plt.title(f'Actual vs Planned Path for {trajectory}')
        plt.xlabel('X Coordinate')
        plt.ylabel('Y Coordinate')
        plt.legend()

        # Save the plot as PNG in a BytesIO object
        fig = plt.gcf()
        buf, filename = save_plot_as_png(fig, f'plot_{trajectory}')
        plots.append(buf)
        plt.close()


    return plots

def main():
    
  
    # Get the name of config file of the current experiment
    specs = os.environ['PARAMS_FILE']
    # Open config file and extact data
    with open(specs, 'r') as file:
        robot_specs = yaml.safe_load(file)
    controller_type=robot_specs['controller_type']
    planner_type=robot_specs['planner_type']    
    pdf_name=robot_specs['experiment_name']
    results_directory=robot_specs['results_directory']
    instances_num= robot_specs['instances_num']
    map_path=robot_specs['map_path']
    map_png_path=robot_specs['map_png_path']
    criteria= robot_specs['criteria']
    weights= robot_specs['weights']
    # Load the CSV file to inspect its contents and structure
    file_path = os.path.join(get_package_share_directory('ROSNavBench'),'raw_data',
        pdf_name+'.csv')
    df = pd.read_csv(file_path)
   
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
    ###Table
    #path_length=calculate_path_length(df)
    path_length=calculate_complete_path_length(df)
    path_deviation=calculate_path_deviation(df)
    #print("PPPPPPPPPPPath lengt",path_length)
    unique_trajectories=df['Trajectory_Type'].unique()
    analysis_data=[]
    
    for i in range(len(planner_type)):

        d=shapes.Drawing(250,40)
        d.add(String(1,20,"-Global planner: "+planner_type[i],fontSize=12,fontName= 'Times-Bold'))
        elements.append(d)
        table= [["Controller\ntype","Trajectory\n type","Success\n rate\n (%)","Average \nexecution\ntime (sec)","CPU(%)","","Memory\n usage(%)","Memory\n usage (%)","Average\nnumber of\nrecoveries","Average\n path\nlength(m)","Min\ndistance to\nobstacles(m)","Average\nPath\n Deviation"]]
        table.append(["","","","","Mean","Max","Mean","Max","","","",""])
        for k in range(len(controller_type)):
            for p in range(len(unique_trajectories)):

                filtered_data = df[(df['Trajectory_Type'] == unique_trajectories[p]) & (df['Planner'] == planner_type[i]) & (df['Controller'] == controller_type[k])]

                table_data=[]
                table_data.append(controller_type[k])
                table_data.append(unique_trajectories[p])

                results_experiment=filtered_data[(filtered_data['result']!="In progress")]

                table_data.append(str(round((results_experiment['result'].value_counts().get('succeeded', 0)/len(results_experiment['result']))*100,2))) #success rate of this combination
                
                table_data.append(str('{0:.2f}'.format(results_experiment['Navigation_time'].mean())))    #Execution time ####

                table_data.append(str('{0:.2f}'.format(filtered_data['CPU_Usage'].mean())))  # average CPU

                print("print the df")
                print(df['CPU_Usage'].head())
                print("print the filttered")
                print(filtered_data['CPU_Usage'].head())

                table_data.append(str(filtered_data['CPU_Usage'].max()))    #Max CPU
                table_data.append(str('{0:.2f}'.format(np.mean(filtered_data['Memory_Usage'].mean())))) #average Memory
                table_data.append(str(filtered_data['Memory_Usage'].max()))  #Max memory
                table_data.append(str('{0:.2f}'.format(results_experiment['number_of_recoveries'].mean())))    #Number of recoveries
                path_length_=path_length[(path_length['Trajectory_Type'] == unique_trajectories[p])&(path_length['Planner'] == planner_type[i])&(path_length['Controller'] == controller_type[k])]
                table_data.append(str('{0:.2f}'.format(path_length_['distance'].mean()))) #Path length    ###
                table_data.append(str(round(filtered_data['distance_to_obstacles'].min(),2)))  #Proximity to obstacles
                path_deviation_=path_deviation[(path_deviation['Trajectory_Type'] == unique_trajectories[p])&(path_deviation['Planner'] == planner_type[i])&(path_deviation['Controller'] == controller_type[k])]
                table_data.append(str(round(path_deviation_['deviation'].mean(),2)))

                table.append(table_data)
                print("cpu column",filtered_data['CPU_Usage'].head())
                
            
                analysis_data.append(table_data.copy())
                analysis_data[len(analysis_data)-1].append(planner_type[i])
                
       
        t=Table(table)
        t.setStyle(TableStyle([('INNERGRID',(0,0), (-1,-1), 0.25, colors.black),
                            ('BOX',(0,0), (-1,-1), 0.25, colors.black),
                            ('SPAN',(0,0),(0,1)),
                            ('SPAN',(1,0),(1,1)),
                            ('SPAN',(2,0),(2,1)),
                            ('SPAN',(3,0),(3,1)),
                            ('SPAN',(8,0),(8,1)),
                            ('SPAN',(9,0),(9,1)),
                            ('SPAN',(10,0),(10,1)),
                            ('SPAN',(11,0),(11,1)),
                            ('SPAN',(4,0),(5,0)),
                            ('SPAN',(6,0),(7,0)),
                            ('FONTNAME',(0,0),(11,1),'Times-Bold'),
                            ('FONTSIZE',(0,0),(11,-1),9),
                            ]))
        elements.append(t)
        rows=[]
        for i in range(len(analysis_data)):
        # Safely access each element, considering the length of each list
            row_data = {
                'Trajectory_Type': analysis_data[i][1],
                'Planner': analysis_data[i][12],
                'Controller': analysis_data[i][0],
                'Time': analysis_data[i][3],
                'CPU': analysis_data[i][4],
                'Memory': analysis_data[i][6],
                'number_of_recoveries': analysis_data[i][8],
                'Safety': analysis_data[i][10],
                'path_length':analysis_data[i][9],
                'path_deviation':analysis_data[i][11],
                'success_rate':analysis_data[i][2]
            }
            rows.append(row_data)
        
        # Convert list of dictionaries to DataFrame
        performacne_df = pd.DataFrame(rows)
        #print("AAAAnalaysis data")
        #print(performacne_df)


    # Setting the size of the graphs 
    width,height,label_size,title_size=calculate_plot_size(len(planner_type),len(controller_type))
    width_pt=width*72
    if width_pt<250:
       images_in_row=floor(480/width_pt)
    else:
       images_in_row=1
     
    print("~~~~~~~~SIZE is~~~~  ",width,height)
    # Performace analysis 
    d=shapes.Drawing(250,80)
    d.add(String(1,60,"Performace analysis",fontSize=15)) 
    d.add(String(1,40,"Based on the criteria:" +", ".join(criteria),fontName= 'Times-Bold'))
    scores=performance_analysis(criteria,performacne_df,weights,planner_type,controller_type)
    #d=shapes.Drawing(250,20*(scores.shape[0]+1))
    d.add(String(1,20,"The score of each controller and planner combinations are:",fontName= 'Times-Bold'))
    # for i in range(scores.shape[0]):
    #     d.add(String(1,20*(i+1),scores["Planner"][i]+" "+scores["Controller"][i]+" "+str(round(scores["total_weighted_score"][i],3))))
   
    elements.append(d)
    plt.figure(figsize=(width,height))
    pivot_data = scores.pivot(index="Planner", columns="Controller", values="total_weighted_score")
    print("LABEL SIZE",label_size,"   ",title_size)
    sns.set(font_scale=label_size/12)
    sns.heatmap(pivot_data, annot=True, cmap="YlGnBu", fmt=".3f")
    
    fig = plt.gcf()
    buf, filename = save_plot_as_png(fig, f'Heatmap_Planners_Controllers')
    plt.close()
    #Prepare the data for the analysis 
    # send to the function
    # include in pdf     
    elements.append(Image_pdf(buf,width*72,height*72))
    d=shapes.Drawing(250,40)
    d.add(String(1,20,"Success rate",fontSize=11,fontName= 'Times-Bold')) 
    elements.append(d) 

    final_results = df.groupby(['Experiment_ID', 'Iteration_ID','Planner','Controller'])['result'].last().reset_index()
    d=shapes.Drawing(250,20*(df['Planner'].nunique()+1))
    d.add(String(1,20*(df['Planner'].nunique()+1),"Planners' success rate are:",fontName= 'Times-Bold'))
    row_increament=1
    for i in range(len(planner_type)):
      
        planner_to_filter = planner_type[i]
        filtered_by_planner = final_results[final_results['Planner'] == planner_to_filter]
        count_by_planner = len(filtered_by_planner)

        # Further filter the results for 'succeeded' and count the number of occurrences
        filtered_by_success = filtered_by_planner[filtered_by_planner['result'] == 'succeeded']
        count_by_success = len(filtered_by_success)
        success_rate_percent = (count_by_success/count_by_planner) * 100  # Convert to percentage

        d.add(String(1,20*(row_increament),f"Planner: {planner_to_filter}, Success Rate: {success_rate_percent:.2f}%"))
        row_increament+=1
    elements.append(d)  
    d=shapes.Drawing(250,20*(df['Controller'].nunique()+1))
    d.add(String(1,20*(df['Controller'].nunique()+1),"Controllers' success rate are:",fontName= 'Times-Bold'))
    row_increament=1
 
    for i in range(len(controller_type)):
        controller_to_filter = controller_type[i]
        filtered_by_controller = final_results[final_results['Controller'] == controller_to_filter]
        count_by_controller = len(filtered_by_controller)

        # Further filter the results for 'succeeded' and count the number of occurrences
        filtered_by_success = filtered_by_controller[filtered_by_controller['result'] == 'succeeded']
        count_by_success = len(filtered_by_success)
        success_rate_percent = (count_by_success/count_by_controller) * 100  # Convert to percentage
     
        d.add(String(1,20*(row_increament),f"Controller: {controller_to_filter}, Success Rate: {success_rate_percent:.2f}%"))
        row_increament+=1    
    
    elements.append(d)  
    ###Graphs
    ##barchart 
    images_list=[]
    for i in ["Navigation_time","CPU_Usage","Memory_Usage","number_of_recoveries","distance_to_obstacles"]:
        average_per_combination_png = create_and_save_bar_chart(df, i,width,height)
        #new_barchart=resize_image_to_max_width(average_per_combination_png[0],400) 
        #elements.append(Image_pdf(new_barchart[0],new_barchart[1][0],int(new_barchart[1][1])))
        images_list.append(average_per_combination_png[0])
        #if len(images_list)==images_in_row:
        #   new_image=combine_images(images_list[0],images_list[1])
        #   elements.append(Image_pdf(new_image,width*72*images_in_row,height*72)) 
    combined_images,image_length=combine_matrix_images(images_list,images_in_row) 
    for i in range(len(combined_images)): 
        elements.append(Image_pdf(combined_images[i],width*image_length[i]*72,height*72))

        #if i=="distance_to_obstacles" and images_in_row!=1:
        #   elements.append(Image_pdf(images_list[0],width*72,height*72))  

    ##Heatmap
    images_list=[]
    for i in ["Navigation_time","CPU_Usage","Memory_Usage","number_of_recoveries","distance_to_obstacles"]:
        heatmap_png = create_and_save_heatmap(df, i,width,height)
        #new_heatmap=resize_image_to_max_width(heatmap_png[0],400) 
        #elements.append(Image_pdf(new_heatmap[0],new_heatmap[1][0],int(new_heatmap[1][1])))
        #elements.append(Image_pdf(heatmap_png[0],width*72,height*72))
        images_list.append(heatmap_png[0])
    combined_images,image_length=combine_matrix_images(images_list,images_in_row) 
    print("LENNNGTTTH",image_length)
    for i in range(len(combined_images)): 
        elements.append(Image_pdf(combined_images[i],width*image_length[i]*72,height*72))    
    #if len(images_list)==images_in_row:
    #    new_image=combine_images(images_list[0],images_list[1])
    #    elements.append(Image_pdf(new_image,width*72*images_in_row,height*72)) 
 
        #if i=="distance_to_obstacles" and images_in_row!=1:
        #   elements.append(Image_pdf(images_list[0],width*72,height*72))  

    ##boxplot
    for i in ["Navigation_time","CPU_Usage","Memory_Usage","number_of_recoveries","distance_to_obstacles"]:
        if i=="Navigation_time" or i=="number_of_recoveries":
            filtered_results=df[(df['result']!="In progress")]
            boxplot,filename,plot_height_,plot_width_=plot_metric_distribution_complex_boxplot(filtered_results, i)
        else:
            boxplot,filename,plot_height_,plot_width_=plot_metric_distribution_complex_boxplot(df, i)
        #new_box_plot=resize_image_to_max_width(boxplot[0],500) 
        #elements.append(Image_pdf(new_box_plot[0],new_box_plot[1][0],int(new_box_plot[1][1])))
        elements.append(Image_pdf(boxplot,plot_width_*72,plot_height_*72))
    path_deviation=plot_path_deviation_heatmap(df,width,height)
    elements.append(Image_pdf(path_deviation[0],width*72,height*72))
    path_length=create_path_length_barchart(df,width,height)
    elements.append(Image_pdf(path_length[0],width*72,height*72))
    unique_trajectories = df['Trajectory_Type'].unique()
    items = 'red blue yellow pink black aqua bisque blueviolet brown burlywood cadetblue chartreuse chocolate cornflowerblue crimson cyan green darkblue darkcyan darkgoldenrod darkgray coral darkgreen darkkhaki darkmagenta darkolivegreen darkorange darkred darksalmon darkseagreen darkslateblue darkslategray darkturquoise darkviolet deeppink deepskyblue dimgray firebrick forestgreen fuchsia grey greenyellow gold hotpink indianred ivory lavender lime maroon navy olive'.split()
    csv_trajectory_path=os.path.join(get_package_share_directory('ROSNavBench'),
        'raw_data',pdf_name+'_trajectories.csv')
    trajectories = pd.read_csv(csv_trajectory_path)  
    print("trajectories are",trajectories)
    unique_initial_trajectories = trajectories['traj_type'].unique()
    sns.set(font_scale=1)
    for i in range(len(planner_type)):
        for j in range(len(unique_trajectories)):
            # start the graph
            #Open the map
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
                print("here1")
                
                ax.imshow(ndimage.rotate(img,90),cmap=plt.cm.gray,extent=[ resolution*x_length+origin[1],origin[1],origin[0], resolution*y_length+origin[0]])
                for k in range(len(controller_type)):

                    # round_num=i*len(controller_type)*instances_num+k*instances_num
                    # x_array=x_points[round_num:round_num+instances_num]
                    # y_array=y_points[round_num:round_num+instances_num]

                    # Filter by j type and number, e.g., 'circle', 2
                    filtered_df = df[(df['Trajectory_Type'] == unique_trajectories[j]) & (df['Planner'] == planner_type[i])& (df['Controller'] == controller_type[k])]
                    plt.scatter(filtered_df['y_pose'].tolist(),filtered_df['x_pose'].tolist(),color=items[k],alpha=0.5)
                    labels_tag.append(controller_type[k])
                plt.xlabel('y-axis (meters)',fontdict={'family':'serif','size':12})
                plt.ylabel('x-axis (meters)',fontdict={'family':'serif','size':12})
                # Filter by trajectory type and number, e.g., 'circle', 2
                

                #  plot the global path planner
                #reset_df = filtered_df.reset_index(drop=True)

                x_path_plan = filtered_df['x_path_plan'].tolist()
                y_path_plan = filtered_df['y_path_plan'].tolist()
                plt.scatter(y_path_plan,x_path_plan,c='yellowgreen',marker='x')

                # plot way points
                # Filter by trajectory type and number, e.g., 'circle', 2
                filtered_traj = trajectories[(trajectories['traj_type'] == unique_initial_trajectories[j]) & (trajectories ['traj_number'] == j)]
                #  Plot the initial pose
                x=filtered_traj['spawn_x'].unique()
                y=filtered_traj['spawn_y'].unique()
                x=x[0]
                y=y[0]
                plt.plot(y,x,marker='*',c='yellowgreen',ms=13)
                # Select final_y column
                y_trajectory = filtered_traj['final_y'].values
                x_trajectory = filtered_traj['final_x'].values
                plt.scatter(y_trajectory,x_trajectory,marker='*',c='yellowgreen')
                y_length=len(numpydata)
                x_length=len(numpydata[0]) # width
            else:
                print("here")
                
                ax.imshow(img,cmap=plt.cm.gray,extent=[origin[0],0.05*y_length+origin[0],origin[1],0.05*x_length+origin[1]])
                for k in range(len(controller_type)):
                    # # round_num=i*len(controller_type)*instances_num+k*instances_num
                    # # x_array=x_points[round_num:round_num+instances_num]
                    # # y_array=y_points[round_num:round_num+instances_num]
                    # Filter by j type and number, e.g., 'circle', 2
                    filtered_df = df[(df['Trajectory_Type'] == unique_trajectories[j]) & (df['Planner'] == planner_type[i])& (df['Controller'] == controller_type[k])]
                
                    plt.scatter(filtered_df['x_pose'].tolist(),filtered_df['y_pose'].tolist(),color=items[k],alpha=0.5)
                    labels_tag.append(controller_type[k])
      
                plt.xlabel('x-axis (meters)',fontdict={'family':'serif','size':12})
                plt.ylabel('y-axis (meters)',fontdict={'family':'serif','size':12})
                # Filter by trajectory type and number, e.g., 'circle', 2
                filtered_traj = trajectories[(trajectories['traj_type'] == unique_initial_trajectories[j]) & (trajectories ['traj_number'] == j)]
                #  Plot the initial pose
                x=filtered_traj['spawn_x'].unique()
                y=filtered_traj['spawn_y'].unique()
                x=x[0]
                y=y[0]
                plt.plot(x,y,marker='*',c='yellowgreen',ms=13)
                
                #  plot the global path planner
                x_path_plan = filtered_df['x_path_plan'].tolist()
                y_path_plan = filtered_df['y_path_plan'].tolist()
                #print("x plaaan",x_path_plan)
                plt.scatter(x_path_plan,y_path_plan,c='yellowgreen',marker='x')
                # plot way points
                

                # Select final_y column
                y_trajectory = filtered_traj['final_y'].values
                x_trajectory = filtered_traj['final_x'].values
                plt.scatter(x_trajectory,y_trajectory,marker='*',c='yellowgreen')

            #inner loop plot a new line
            plt.legend(labels_tag,prop={'family':'serif','size':8})
            plt.savefig(os.path.join(get_package_share_directory('ROSNavBench'),
            'raw_data','map_plot'+str(i)+str(j)+'.png'))
            
            #rescaling the image
            max_width = 400
            if x_length != max_width:
                scaling_factor = max_width / x_length
                x_length = int(x_length * scaling_factor)
                y_length = int(y_length * scaling_factor)

            drawing = shapes.Drawing(500,60)
            drawing.add(String(1,50,"-Global planner: "+planner_type[i],fontSize=12,fontName= 'Times-Bold'))
            drawing.add(String(300,50,"-Trajectory type: "+unique_trajectories[j],fontSize=12,fontName= 'Times-Bold'))
            drawing.add(String(200,40,'Traveled path ', fontSize=12, fillColor=colors.black))
            #drawing.add(Circle(300,10,3,fillColor=colors.white))
            #drawing.add(String(308,10,'Final pose ',fontSize=12, fillColor=colors.black))
            star_vertices=[201.99,9.78,205.88,11.194,201.04,11.708,200,16,198.8,11.6,194.1,11.091,198.07,9.46,196.3,5.277,200,8,203.86,5.406,201.99,9.78]
            drawing.add(Polygon(star_vertices, fillColor=colors.yellowgreen,strokeColor=colors.yellowgreen))
            drawing.add(String(210,10,'Initial pose ',fontSize=12, fillColor=colors.black))
            #drawing.add(Line(1,13,7,13, strokeColor=colors.yellowgreen, strokeWidth=3))
            drawing.add(String(1,11,'x',fontSize=12, fillColor=colors.yellowgreen))
            drawing.add(String(9,10,'Global planner path ',fontSize=12, fillColor=colors.black))
            drawing.add(String(115,10,'* ',fontSize=13, fillColor=colors.yellowgreen))
            drawing.add(String(124,10,'Waypoints ',fontSize=12, fillColor=colors.black))

            elements.append(drawing)
            elements.append(Image_pdf(os.path.join(get_package_share_directory('ROSNavBench'),
            'raw_data','map_plot'+str(i)+str(j)+'.png'),y_length,x_length))
  

    final_results = df.groupby(['Experiment_ID', 'Iteration_ID'])['result'].last().reset_index()

    # Filter to include only trails where the final result is not 'succeeded'
    trails_not_succeeded = final_results[final_results['result'] != 'succeeded']
   
    # Merge this with the original data to get the full data for these trails
    filtered_trails_data = df.merge(trails_not_succeeded, on=['Experiment_ID', 'Iteration_ID'], how='inner')

    # Display the first few rows of the filtered trails data
    print("failed",filtered_trails_data.head())
    if not filtered_trails_data.empty:
            d=shapes.Drawing(250,60)
            d.add(String(1,40,"Failure report",fontSize=15))
            
            d.add(String(1,20," Recorded log messages of navigation nodes, if any message is recorded",fontSize=10))
            
            elements.append(d)
  
    for (experiment_id, iteration_id), trail_data in filtered_trails_data.groupby(['Experiment_ID', 'Iteration_ID']):
        # Processing each trail
        log_msgs=[]
        table= [["Global planner: "+str(trail_data['Planner'].unique()[0]),'',''],["Controller:  "+str(trail_data['Controller'].unique()[0])+'    experiment#:'+str(trail_data['Experiment_ID'].unique()[0])+'    iteration#:'+str(trail_data['Iteration_ID'].unique()[0]),'',''],["Logger_name", "Level", "Message"]]
        table.append([""])
        print("msg_level",pd.isna(trail_data['msg_level']))
        print("Error_msgs",pd.isna(trail_data['Error_msgs']))
        for index, row in trail_data.iterrows():
            # Check for non-NaN values and process them
            
            #if not pd.isna(row['publisher_node']) and not pd.isna(row['msg_level']) and not pd.isna(row['Error_msgs']):
            if  not pd.isna(row['msg_level']) and not pd.isna(row['Error_msgs']):
             
                # Process the non-NaN values
                # For example: print(row['publisher_node'], row['msg_level'], row['Error_msgs'])
                log_msgs=[row["publisher_node"],row["msg_level"],row["Error_msgs"]]
                table.append(log_msgs)


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
        #if pd.isna(trail_data['msg_level']) and pd.isna(trail_data['Error_msgs']):
        #    d=shapes.Drawing(250,25)
        #    d.add(String(1,20,"Global planner: "+str(trail_data['Planner'].unique()[0])+", Controller:  "+str(trail_data['Controller'].unique()[0])    +', experiment#:'+str(trail_data['Experiment_ID'].unique()[0])+', iteration#:'+str(trail_data['Iteration_ID'].unique()[0])+ ", No log messages recorded of level error or warn",fontSize=10))
        #    elements.append(d)  
    doc.build(elements)

