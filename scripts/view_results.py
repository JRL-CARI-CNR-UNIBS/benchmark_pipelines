#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""

author: jacobi
"""

import yaml
import numpy  as np
import math
import matplotlib.pyplot as plt
import seaborn
import pandas as pd

load_from_parameter_server=True
for dof in [6]: #[6, 9,12,18]:
    test_name='benchmark_result_'+str(dof)+'b.yaml'

    if load_from_parameter_server:
       import rospy
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

    cost_over_time=[]
    planner_failures=[]


    #tested_planners=tested_planners[0:2]
    #planner_name=["Informed RRT*","Mixed-strategy RRT*"]

    tested_planners=tested_planners[0:3]
    planner_name=["Informed RRT*","Mixed-strategy RRT*","Wrap RRT*",]

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
    sel_planning_times=[0.05,0.2,0.5,1,3,5]
    db2=db1.loc[db1['Planning time [s]'].isin(sel_planning_times),:]
    #fig, ax = plt.subplots(figsize=a4_dims);
    #seaborn.violinplot(x="Planning time [s]",y="Normalized length",hue="planner",data=db2,split=True, inner=None,cut=False,scale="width")
    #plt.grid()
    #fig.savefig("violin_dof"+str(dof)+".png",dpi=300)

    fig, ax = plt.subplots(figsize=a4_dims);
    seaborn.set_context("paper", rc={"font.size":18,"axes.titlesize":18,"axes.labelsize":18,"legend.fontsize":18, "xtick.labelsize": 18, "ytick.labelsize":18})
    g=seaborn.boxenplot(hue="planner",y="Normalized length",data=db2,x="Planning time [s]", palette="deep")
    g.grid()
    fig.savefig("boxen_dof"+str(dof)+".png",dpi=300, bbox_inches = 'tight')
