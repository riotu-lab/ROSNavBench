import os
import math
import yaml
import csv
import numpy as np
from matplotlib import pyplot as plt
from ament_index_python.packages import get_package_share_directory
from reportlab.lib.pagesizes import A4
from reportlab.graphics.shapes import *
from reportlab.graphics.charts.axes import XCategoryAxis, YCategoryAxis
from reportlab.graphics.charts.lineplots import LinePlot
from reportlab.graphics.widgets.markers import makeMarker
from reportlab.platypus import SimpleDocTemplate, Table, TableStyle
from reportlab.lib import colors
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.enums import TA_CENTER

def main():
    
    def generate_list(min_val, max_val, step_size):
        generated_list = []
        current_val = min_val
        while current_val <= max_val:
            generated_list.append(current_val)
            current_val += step_size
        return generated_list
    
    def read_controller_data(file_name):
        data = []
        with open(file_name, 'r') as csvfile:
            reader = csv.reader(csvfile)
            for row in reader:
                data.append(row)
        return data
    
    # Get the name of the config file of the current experiment
    params_file = os.environ['PARAMS_FILE']
    
    # Open config file and extract data
    specs = os.path.join(
        get_package_share_directory('ROSNavBench'),
        'config',
        params_file + '.yaml'
    )
    with open(specs, 'r') as file:
        robot_specs = yaml.safe_load(file)
    
    controller_type = robot_specs['controller_type']
    results_directory = robot_specs['results_directory']

    trails_num = robot_specs['trails_num']
    
    if trails_num > 0:
        controller_type = [controller_type[0]] * trails_num
    
    data = []
    CPU_data = []
    global_CPU = []

    for _ in range(len(controller_type)):
        data.append([])
    
    for i in range(len(controller_type)):
        file_name = os.path.join(results_directory, controller_type[i] + '.csv')
        data[i] = read_controller_data(file_name)
    
    for i in range(len(controller_type)):
        CPU_data.append([float(row[3]) for row in data[i]])
    
    global_CPU = [np.mean(CPU_data[i]) for i in range(len(controller_type))]
    max_CPU = [max(CPU_data[i]) for i in range(len(controller_type))]
    min_CPU = [min(CPU_data[i]) for i in range(len(controller_type))]
    
    x_axis_values = generate_list(min_CPU[0], max_CPU[0], 1)
    
    ax = plt.subplots(figsize=(10, 6))  # Adjust the figure size
    for i in range(len(controller_type)):
        ax.plot(x_axis_values, CPU_data[i], label=controller_type[i])

    plt.title('Comparison of Controllers: CPU Usage')
    plt.xlabel('CPU Usage (%)')
    plt.ylabel('Controller')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()  # Add this to improve spacing
    plt.savefig('CPU_usage.png')

    # Organize table data for better alignment
    table_data = [
        ['Controller', 'Average CPU Usage', 'Maximum CPU Usage', 'Minimum CPU Usage'],
    ]
    for i in range(len(controller_type)):
        table_data.append([
            controller_type[i],
            '{:.2f}'.format(global_CPU[i]),
            '{:.2f}'.format(max_CPU[i]),
            '{:.2f}'.format(min_CPU[i])
        ])

    table = Table(table_data)
    
    table_style = TableStyle()
    table_style.setFont('Times-Roman', 12)
    table_style.setAlignment(TA_CENTER)
    table_style.addBorder(1, colors.black)
    table_style.setFillColor('lightgrey')
    table.setStyle(table_style)

    doc = SimpleDocTemplate('CPU_usage_table.pdf', pagesize=A4)
    doc.build([table])  # Wrap the table in a list to prevent extra space
    
if __name__ == "__main__":
    main()