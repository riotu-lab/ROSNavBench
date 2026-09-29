#! /usr/bin/env python3

"""
Multi-criteria scoring of planner--controller combinations.
Implements the Data Analysis Block of the ROSNavBench paper:
  * Min--Max normalisation per trajectory type      (Eqs. 2 and 3)
import string
  * Weight constraint: sum(alpha) = 1, 0 < alpha <= 1 (Eq. 4)
  * Rank-order weights from the criteria order     (Eq. 5)
  * User-given scores normalised to weights        (Eq. 6)
  * Weighted sum, summed over trajectory types     (Eq. 7)
  * Best configuration = highest score             (Eq. 8)

"""
#criteria=["Time","CPU","Path Length","Safety","Memory"]
#List of possible criteria 
#Time
#Path Length
#Safety in terms of the closest point to the obstacles
#Computation: involves both CPU and memory >the user decide which one to include
# Above criteria will be only of suessful iterations, failed iterations will be excluded
# success rate will be provided on seperate analysis   

#Data will be recived as a table of data [[name of critera as in table],[1m,1,1,1,]]

from itertools import combinations
import string
import numpy as np
import pandas as pd



def performance_analysis(criteria,data,weights,planner_type,controller_type):

    if weights is None or str(weights).strip().lower() in ('none', 'null', '', '[]'):
        #The user have not spceified weights
        normalized_weights=assign_weight(criteria)
    else:
        #The user have spceified weights in form of giving a weight for each criteria from 1 to 9 
        normalized_weights=convert_weight(criteria,weights)
    normalized_data=normalize_by_trajectory_type(data)
    scores=calculate_weighted_average(normalized_data, criteria, normalized_weights)


    return scores


def convert_weight(criteria_order,weights):
    # if the user spcify the weight, the weights are normalized using this function 
    total_weights=sum(weights)
    for i in range(len(weights)):
        weights[i]=weights[i]/total_weights
    weights=dict(zip(criteria_order,weights))
    return weights
    
def assign_weight(criteria_order):
    # Assigning normalized weights to each criteria based on their order 
    # Create a dictionary to store criteria and their assigned weights
    weights = {}
    num_criteria = len(criteria_order)
    
    
   
    # Generate weights based on the order provided by the user
    for i, criterion in enumerate(criteria_order):
        weight = (num_criteria - i) / sum(range(1, num_criteria + 1))
        weights[criterion] = weight
    # Normalize the weights
    total_weight = sum(weights.values())
    normalized_weights = {criterion: weight / total_weight for criterion, weight in weights.items()}
    normalized_weights

    return normalized_weights

    return normalized_data
def normalize_by_trajectory_type(data):
    normalized_data = data.copy()
    maximized_metrics=["distance_to_obstacles","success_rate","Safety"]
    minimaized_metrics=["Time","CPU","Memory", "path_length", 'path_deviation',  'number_of_recoveries']
    numeric_columns = ["Time","CPU","Memory", "path_length", "Safety",  'path_deviation', 'success_rate', 'number_of_recoveries']

    # Convert to numeric and handle non-numeric values
    for col in numeric_columns:
        normalized_data[col] = pd.to_numeric(normalized_data[col], errors='coerce')

    for traj_type in data['Trajectory_Type'].unique():
        group_data = normalized_data[normalized_data['Trajectory_Type'] == traj_type]
        min_vals = group_data[numeric_columns].min()
        max_vals = group_data[numeric_columns].max()

        range_vals = max_vals - min_vals
        range_vals[range_vals == 0] = 1

        normalized_group_data = (max_vals - group_data[numeric_columns]) / range_vals
        normalized_group_maximaize=(group_data[numeric_columns]-min_vals) / range_vals
        normalized_group_minimize=(max_vals - group_data[numeric_columns]) / range_vals
        normalized_data.loc[normalized_data['Trajectory_Type'] == traj_type, maximized_metrics] = normalized_group_maximaize
        normalized_data.loc[normalized_data['Trajectory_Type'] == traj_type, minimaized_metrics] = normalized_group_minimize
    #print(normalized_data.dtypes)
    return normalized_data


def calculate_weighted_average(data, metrics, weights_dict):
    #print(data.dtypes)

   

    # Check if all metrics are present in the weights dictionary
    if not all(metric in weights_dict for metric in metrics):
        raise ValueError("All metrics must have an associated weight in the weights dictionary")

    # Check that all metrics are numeric columns in the DataFrame
    if not all(metric in data.select_dtypes(include=['float64', 'int64']).columns for metric in metrics):
        raise ValueError("All metrics must be numeric columns in the DataFrame")

    # Create a new column for the weighted score for each row
    for metric in metrics:
        weight = weights_dict[metric]



        data[f'weighted_{metric}'] = data[metric] * weight

    # Calculate the sum of weighted scores for each metric
    data['total_weighted_score'] = data[[f'weighted_{metric}' for metric in metrics]].sum(axis=1)

    # Group by Planner, Controller, and Trajectory_Type and calculate the average weighted score
    group_average = data.groupby(['Planner', 'Controller', 'Trajectory_Type'])['total_weighted_score'].mean().reset_index()

    # Further group by Planner and Controller, and sum the average scores across trajectory types
    total_scores = group_average.groupby(['Planner', 'Controller'])['total_weighted_score'].sum().reset_index()

    # Sort the results in descending order of the total weighted score
    sorted_total_scores = total_scores.sort_values(by='total_weighted_score', ascending=False)

    return sorted_total_scores
    

 
