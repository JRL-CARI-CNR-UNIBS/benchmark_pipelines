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


load_from_parameter_server=True

if load_from_parameter_server:
   param=rospy.get_param("/benchmark")
else:
   with open('../config/result_query01_3dof.yaml') as f:
       param = yaml.safe_load(f)

queries_number=param["queries_number"]
repetitions=param["repetitions"]
query_prefix=param["query_test"]
pipeline_ids=param["pipeline_ids"]

planning_times=param["planning_times"]
# planning_times=planning_times[0:3]
tested_planners=[]
planner_str=[]
for pipeline_id in pipeline_ids:
    planner_ids=param["planner_ids"][pipeline_id]
    for planner_id in planner_ids:
        tested_planners.append([pipeline_id,planner_id])
        planner_str.append(planner_id)

tot_min_length=0
tot_median_length=np.empty([len(tested_planners),len(planning_times)])
tot_median_length[:]=0

tot_failures=np.zeros([len(tested_planners),len(planning_times)])
tot_planning_time=np.zeros([len(tested_planners),len(planning_times)])
marker=[]
for iquery in range(0,queries_number):
    query_str="query_"+str(iquery)
    q=param[query_prefix][query_str];


    min_length=math.inf
    median_length=np.empty([len(tested_planners),len(planning_times)])
    median_length[:]=np.Inf
    failures=np.zeros([len(tested_planners),len(planning_times)])
    query_planning_time=np.zeros([len(tested_planners),len(planning_times)])

    for iplanner in range(0,len(tested_planners)):
        pipeline=tested_planners[iplanner][0]
        planner=tested_planners[iplanner][1]
        if (pipeline=="dirrt"):
            marker.append("d")
        else:
            marker.append("o")
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
                    tot_planning_time[iplanner][iplan_time]+=result["planning_time"]
                    query_planning_time[iplanner][iplan_time]+=result["planning_time"]
                except:
                    trajectory_length[iteration]=np.inf
                    tot_planning_time[iplanner][iplan_time]=np.inf
                    query_planning_time[iplanner][iplan_time]=np.inf
            min_length=min(min_length,np.min(trajectory_length))
            median_length[iplanner][iplan_time]=np.median(trajectory_length)
            tot_median_length[iplanner][iplan_time]+=median_length[iplanner][iplan_time]
            tot_failures[iplanner][iplan_time]+=failures[iplanner][iplan_time]
    fig, axs = plt.subplots(2)
    fig.set_size_inches(10,10)
    for iplanner in range(0,len(tested_planners)):
        #axs[0].semilogy(query_planning_time[iplanner]/repetitions,np.log10(median_length[iplanner]/min_length),"-"+marker[iplanner], label=planner_str[iplanner])
        axs[0].semilogy(query_planning_time[iplanner]/repetitions,(median_length[iplanner]/min_length),"-"+marker[iplanner], label=planner_str[iplanner])
        axs[1].plot(np.array(planning_times),failures[iplanner],marker[iplanner], label=planner_str[iplanner])
    axs[0].legend()
    axs[0].set(xlabel="Max planning time",ylabel="Path length/min(path length))",title=str(query_prefix+"_"+query_str))
    axs[0].grid(True)
    axs[1].set(xlabel="Max planning time",ylabel="Failures")
    axs[1].legend()
    axs[1].grid(True)
    fig.savefig(query_prefix+query_str+".png",dpi=300)
    # plt.show(block=False)
    # plt.pause(1)
    # plt.close()

    tot_min_length+=min_length


fig, axs = plt.subplots(2)
fig.set_size_inches(10,10)
for iplanner in range(0,len(tested_planners)):
   axs[0].semilogy(tot_planning_time[iplanner]/queries_number/repetitions,(tot_median_length[iplanner]/tot_min_length),"-"+marker[iplanner], label=planner_str[iplanner])
   #axs[0].semilogy(tot_planning_time[iplanner]/queries_number/repetitions,np.log10(tot_median_length[iplanner]/tot_min_length),"-"+marker[iplanner], label=planner_str[iplanner])
   axs[1].plot(np.array(planning_times),tot_failures[iplanner],marker[iplanner], label=planner_str[iplanner])
axs[0].legend()
axs[0].set(xlabel="Max planning time",ylabel="Path length/min(path length)",title=str(query_prefix+"_sum"))
axs[0].grid(True)
axs[1].set(xlabel="Max planning time",ylabel="Failures")
axs[1].legend()
axs[1].grid(True)

fig.savefig(query_prefix+"_sum"+".png",dpi=300)
# plt.show(block=False)
# plt.pause(1)
# plt.close()


# fig, axs = plt.subplots(1)
# fig.set_size_inches(10,10)
# for iplanner in range(0,len(tested_planners)):
#    axs.plot(np.array(planning_times),tot_planning_time[iplanner]/queries_number/repetitions,"-"+marker[iplanner], label=planner_str[iplanner])
# axs.legend()
# axs.set(xlabel="Max planning time",ylabel="Planning time",title=str(query_prefix+"_sum"))
# axs.grid(True)
#
# fig.savefig(query_prefix+"_planning_time"+".png",dpi=300)
# plt.show()
