#! /usr/bin/env python3

from itertools import combinations
import string
import numpy as np

criteria=["Time","Computation","Path length"]
#List of possible criteria 
#Time
#Path Length
#Safety in terms of the closest point to the obstacles
#Path tracking in terms of deviation of intial global path and exectued path
#Computation: involves both CPU and memory >the user decide which one to include
# 
# Above criteria will be only of suessful iterations
# success rate will be provided on seperate analysis   

#Data will be recived as a table of data [[name of critera as in table],[1m,1,1,1,]]

def performance_analysis(criteria,data,weights,planner_type,controller_type):
    print("Before ",data)
    data,combinations,iterations_result=extract_data(criteria,data,planner_type,controller_type)
    print("data   ",data)
    print("combinations",combinations)
    print("iterations_result",iterations_result)
    if weights==None:
        #The user have not spceified weights
        normalized_weights=assign_weight(criteria)
    else:
        #The user have spceified weights in form of giving a weight for each criteria from 1 to 9 
        normalized_weights=convert_weight(criteria,weights)
    print("norm weight",normalized_weights)
    normalized_data=normalizing_data(data)
    print("norm data",normalized_data)
    ranking=rank(normalized_data,normalized_weights,combinations,iterations_result)
    print("rank",ranking)
    print(success_rate(iterations_result,controller_type,planner_type))
    planners_success_rate,controllers_success_rate=success_rate(iterations_result,controller_type,planner_type)
    #conclusion=
    return ranking,planners_success_rate,controllers_success_rate

def performance_analysis_repeatability(data,planner_type,controller_type):
    print("Before ",data)
    criteria=["Time","CPU","Path Length","Safety","Memory"]
    
    data,combinations,iterations_result=extract_data(criteria,data,planner_type,controller_type)
    print("data   ",data)
    print("combinations",combinations)
    print("iterations_result",iterations_result)
    # if weights==None:
    #     #The user have not spceified weights
    #     normalized_weights=assign_weight(criteria)
    # else:
    #     #The user have spceified weights in form of giving a weight for each criteria from 1 to 9 
    #     normalized_weights=convert_weight(weights)
    #print("norm weight",normalized_weights)
    #normalized_data=normalizing_data(data)
    #print("norm data",normalized_data)
    #ranking=rank(normalized_data,normalized_weights,combinations)
    #print("rank",ranking)
    print(repatability_success_rate(iterations_result))
    #print(data_variation(data))
    variation=data_variation(data)
    success_rate=repatability_success_rate(iterations_result)
    #conclusion=
    #return ranking
    return variation,success_rate

def extract_data(criteria, data,planner_type,controller_type):
    #This function extract data with respect to the user citeria
    arranged_data=[criteria]
    combinations=[inner_list[0] for outer_list in data for inner_list in outer_list[2:]]
    print(combinations)
    for i in range(len(planner_type)):
        for j in range(len(controller_type)):
            iteration=len(controller_type)*i+j
            combinations[iteration]+=" "+planner_type[i]
       
    iterations_results=[inner_list[1] for outer_list in data for inner_list in outer_list[2:]]
    for i in range(len(criteria)):
        if criteria[i]=="Time":
            #arranged_data.append([sublist[2] for sublist in  data[2:]])
            arranged_data.append([float(inner_list[2]) for outer_list in data for inner_list in outer_list[2:]])
        elif criteria[i]=="CPU": 
            #arranged_data.append([sublist[3] for sublist in  data[2:]])
            arranged_data.append([float(inner_list[3]) for outer_list in data for inner_list in outer_list[2:]])
        elif criteria[i]=="Memory":
            #arranged_data.append([sublist[5] for sublist in  data[2:]])
            arranged_data.append([float(inner_list[5]) for outer_list in data for inner_list in outer_list[2:]])
        elif criteria[i]=="Safety":
            #arranged_data.append([sublist[9] for sublist in  data[2:]]) 
            arranged_data.append([float(inner_list[9]) for outer_list in data for inner_list in outer_list[2:]])
        elif criteria[i]=="Path Length":
            #arranged_data.append([sublist[8] for sublist in  data[2:]])   
            arranged_data.append([float(inner_list[8]) for outer_list in data for inner_list in outer_list[2:]]) 
        #elif criteria[i]=="Path_tracking": 
        #    arranged_data.append([sublist[2] for sublist in  data[1:]])     
    
    
    return arranged_data,combinations,iterations_results

def convert_weight(criteria_order,weights):
    total_weights=sum(weights)
    for i in range(len(weights)):
        weights[i]=weights[i]/total_weights
    weights=dict(zip(criteria_order,weights))
    return weights
    
def assign_weight(criteria_order):
    
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
    


    return normalized_weights



def normalizing_data(data):
    # Normalizing data by min max normalization. 
    # for values where least value is better The equation is Normalized_value=(Max-val)/(Max-Min)
    # for values where highest value is better The equation is Normalized_value=(val-Min)/(Max-Min)

    max_value=[]
    min_value=[]
    for i in range(len(data[0])):
        max_value.append(max(data[i+1]))
        min_value.append(min(data[i+1]))
        
        if not identical_element(data[i+1]):
            print(identical_element(data[i+1]))
            print("i is ",i)
            for j in range(len(data[i+1])):
                if data[0][i]=="Safety": 
                    data[i+1][j]=(data[i+1][j]-min_value[i])/(max_value[i]-min_value[i])
                else: 
                    data[i+1][j]=(max_value[i]-data[i+1][j])/(max_value[i]-min_value[i])
        else:
            # if all values of a criterion are identical , set all of them to zero as they are not going to affect the final result
            data[i+1]=[0]*len(data[i+1])
    
    return data

def identical_element(array):
    ref_element=array[0]
    for i in range(len(array)):
        if array[i] != ref_element:
           return False
    return True 

def rank(normailzed_data,weights,combinations,results):
    for i in range(len(normailzed_data[0])):
        for j in range(len(normailzed_data[i+1])):
            normailzed_data[i+1][j]=round(normailzed_data[i+1][j]*weights[normailzed_data[0][i]]*100,4)
    
    ranking=[]
    #for j in range(len(normailzed_data[i])):
    #    score=0
    #    for i in range(len(criteria)):
    #        score+=normailzed_data[i][j]
    #    ranking.append(score)
    for j in range(len(normailzed_data[1])):
        ranking.append(sum([sublist[j] for sublist in  normailzed_data[1:]])) 
    print(ranking)
    for i in range(len(results)-1,-1,-1):
        if results[i]!='succeeded':
            del combinations[i]
            del ranking[i]
    ranking=dict(zip(combinations,ranking))
    print(ranking)

    ranking=dict(sorted(ranking.items(),key=lambda item: item[1]))
    
    return ranking

def data_variation(data):
    # Finding CPU variation, Memory usage variation, Path length, Execution time, success rate 
    data_variation_summary=[]
    units=[" sec"," %"," m"," m"," %"]
    for i in range(len(data[0])):
        data_variation_summary.append("The "+data[0][i]+" range from "+str(min(data[i+1]))+" to "+str(max(data[i+1]))+units[i])

    return data_variation_summary

def repatability_success_rate(result):
    # successful iterations/total iterations *100
    success_rate_result="The success rate is "+str((result.count('succeeded')/len(result))*100)+"%"
    return success_rate_result
    
def success_rate(result,controller_type,planner_type):
    # this function  calculates the success rate of each planner and controller
    success_rate_planners=''
    success_rate_controllers=''
    controllers=[]
    planners=[]  #[[]]*len(planner_type)
    for i in range(len(planner_type)):
        planners.append("The success rate of "+planner_type[i]+" is "+str((result[i*len(controller_type):((i+1)*len(controller_type))].count('succeeded')/len(controller_type))*100)+" %")
    for i in range(len(controller_type)):   
        controllers.append([])
    for i in range(len(controller_type)):   
        for j in range(len(planner_type)):
        #    iteration=len(controller_type)*i+j
           
            controllers[i].append(result[j*len(controller_type)+i])

    for i in range(len(controllers)):
        controllers[i]="The success rate of "+controller_type[i]+" is "+str((controllers[i].count('succeeded')/len(planner_type))*100)+" %"
    return planners,controllers
            
         
def main():
      
    x=[[["controller","r","exec Time","CPU averag","CPU max","memory ","Memory max","recoveries","Path length","obstacles"],["","","","","","","","",""],["RRr1","succeeded","32.53","17.44","34.4","11.00","11.00","0","6.57","1.5"],["RRr2","failed","32.14","16.95","33.9","11.00","11.0","0","6.55","1.2"],["RRr3","succeeded","31.79","17.50","31.79","11.0","11.1","0","6.56","3.2"]]]
    print(performance_analysis(["Time","CPU","Path Length","Safety","Memory"],x,None,["NavFn"],["RPP","RPP","RPP"]))