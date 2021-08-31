#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""

author: jacobi
"""

import yaml
import rospy
import numpy  as np
import math
import matplotlib.pyplot as plt
import seaborn
import pandas as pd

load_from_parameter_server=False
dof=6
test_name='benchmark_result_'+str(dof)+'.yaml'

if load_from_parameter_server:
   param=rospy.get_param("/benchmark")
else:
   with open(test_name) as f:
       param = yaml.safe_load(f)

queries_number=param["queries_number"]
repetitions=param["repetitions"]
query_prefix=param["query_test"]
pipeline_ids=param["pipeline_ids"]

planning_times=param["planning_times"]
tested_planners=[]
planner_str=[]
for pipeline_id in pipeline_ids:
    planner_ids=param["planner_ids"][pipeline_id]
    for planner_id in planner_ids:
        tested_planners.append([pipeline_id,planner_id])
        planner_str.append(planner_id)

tested_planners=tested_planners[0:2]
cost_over_time=[]
planner_failures=[]


planner_name=["Informed RRT*","Mixed-strategy RRT*"]
for iquery in range(0,queries_number):
    query_str="query_"+str(iquery)
    q=param[query_prefix][query_str];


    min_length=math.inf
    lengths=np.empty([len(tested_planners),len(planning_times),repetitions])
    failures=np.zeros([len(tested_planners),len(planning_times)])

    for iplanner in range(0,len(tested_planners)):
        pipeline=tested_planners[iplanner][0]
        planner=tested_planners[iplanner][1]
        for iplan_time in range(0,len(planning_times)):
            planning_time=planning_times[iplan_time]
            planning_time_string='planning_time_ms_'+str(int(1000*planning_time))
            trajectory_length=np.zeros(repetitions)

            for iteration in range(0,repetitions):
                iteration_str="iteration_"+str(iteration)
                try:
                    result=q[pipeline][planner][iteration_str][planning_time_string]
                    error_code=result["error_code"]
                    if error_code==1:
                        trajectory_length[iteration]=result["trajectory_length"]
                    else:
                        trajectory_length[iteration]=np.inf
                        failures[iplanner][iplan_time]+=1
                except:
                    trajectory_length[iteration]=np.inf
            min_length=min(min_length,np.min(trajectory_length))
            lengths[iplanner][iplan_time]=trajectory_length

    for iplanner in range(0,len(tested_planners)):
        for iplan_time in range(0,len(planning_times)):
            for iteration in range(0,repetitions):
                entry={'Planning time [s]': planning_times[iplan_time],
                   'planner': planner_name[iplanner],
                   'Normalized length': min(lengths[iplanner][iplan_time][iteration]/min_length,10),
                   }
                cost_over_time.append(entry)

a4_dims = (11.7, 8.27)
db1=pd.DataFrame(cost_over_time)
fig, ax = plt.subplots(figsize=a4_dims);
seaborn.violinplot(x="Planning time [s]",y="Normalized length",hue="planner",data=db1,split=True, inner=None,cut=False,scale="count")
plt.grid()
fig.savefig("violin_dof"+str(dof)+".png",dpi=300)
