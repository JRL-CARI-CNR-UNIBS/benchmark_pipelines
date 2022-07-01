#!/usr/bin/env python3
"""
Copyright (c) 2022, JRL-CARI CNR-STIIMA/UNIBS
Manuel Beschi manuel.beschi@unibs.it
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

import rospy
import sys
from std_msgs.msg import Float64MultiArray
import numpy as np
import seaborn as sns
import pandas as pd
import matplotlib.pyplot as plt
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import object_loader_msgs.srv



performance=np.array([])
received=False;
sampling_time=0.1

def performanceCallback(data):
    global array
    global received

    array=np.array(data.data).reshape(-1,3)
    received=True

def runQuery():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.

    global received
    global array

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('run_query', anonymous=True)

    print(rospy.get_name())

    rospy.Subscriber("/solver_performance", Float64MultiArray, performanceCallback)

    rospy.wait_for_service('/add_objects_group')
    obj_loader_srv = rospy.ServiceProxy('/add_objects_group', object_loader_msgs.srv.AddObjectsGroup)
    obj_unloader_srv = rospy.ServiceProxy('/remove_objects_group', object_loader_msgs.srv.RemoveObjectsGroup)

    test_name=rospy.get_param("~test_name")
    group_name=rospy.get_param("~group_name")
    repetitions=rospy.get_param("~repetitions")
    scenarios=rospy.get_param("~scenarios")
    queries_number=rospy.get_param("~queries_number")
    starting_query=rospy.get_param("~starting_query")
    planning_time=rospy.get_param("~planning_time")
    long_planning_time=rospy.get_param("~long_planning_time")
    pipeline_ids=rospy.get_param("~pipeline_ids")
    planner_ids=[]
    for pipeline_id in pipeline_ids:
        tmp=rospy.get_param("~planner_ids/"+pipeline_id)
        for planner_id in tmp:
            planner_ids.append([pipeline_id,planner_id])

    print(planner_ids)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    move_group = moveit_commander.MoveGroupCommander(group_name)
    state=move_group.get_current_state()

    mg_client = actionlib.SimpleActionClient('/move_group', moveit_msgs.msg.MoveGroupAction)
    mg_client.wait_for_server()

    first_run=True
    planning_time=planning_time+0.01


    mg_goal=moveit_msgs.msg.MoveGroupGoal()
    mg_goal.request.group_name=group_name
    mg_goal.request.num_planning_attempts=1
    mg_goal.request.max_velocity_scaling_factor=1.0
    mg_goal.request.max_acceleration_scaling_factor=1.0
    mg_goal.planning_options.plan_only=True


    for scenario in scenarios:

        object_groups = rospy.get_param("~"+scenario+"/object_groups")
        for og in object_groups:
            og_req=object_loader_msgs.srv.AddObjectsGroupRequest()
            og_req.objects_group=og
            obj_loader_srv(og_req)

        for actual_query_number in range(starting_query,queries_number):
            print("query: ",actual_query_number)

            start_configuration = rospy.get_param("~"+scenario+"/query_"+str(actual_query_number)+"/start_configuration")
            goal_configurations  = rospy.get_param("~"+scenario+"/query_"+str(actual_query_number)+"/goal_configurations")

            state.joint_state.position=start_configuration

            mg_goal.request.start_state=state

            mg_goal.request.goal_constraints.clear()

            for ig in range(len(goal_configurations)):

                goal_conf=moveit_msgs.msg.Constraints()
                goal_conf.name="goal"+str(ig)
                goal_configuration=goal_configurations[ig]
                for ijoint in range(len(mg_goal.request.start_state.joint_state.name)):
                    jc=moveit_msgs.msg.JointConstraint()
                    jc.joint_name=mg_goal.request.start_state.joint_state.name[ijoint]
                    jc.position=goal_configuration[ijoint]
                    goal_conf.joint_constraints.append(jc)

                mg_goal.request.goal_constraints.append(goal_conf)

            for pp in planner_ids:
                pipeline_id=pp[0]
                planner_id=pp[1]

                print(pipeline_id,", ",planner_id)
                mg_goal.request.pipeline_id=pipeline_id
                mg_goal.request.planner_id=planner_id

                for rep in range(repetitions):
                    if rospy.is_shutdown():
                        exit()
                    if (rep==0):
                        mg_goal.request.allowed_planning_time=max(long_planning_time,planning_time)
                    else:
                        mg_goal.request.allowed_planning_time=planning_time
                    print("repetition = ",rep)
                    received=False

                    mg_client.send_goal(mg_goal)
                    mg_client.wait_for_result()
                    while (not received):
                        rospy.sleep(0.001)

                    time=array[:,0]
                    itera=array[:,1]
                    cost=array[:,2]
                    t=np.arange(0, planning_time, sampling_time)
                    cost_interp = np.interp(t, time, cost)
                    iter_interp = np.round_(np.interp(t, time, itera))
                    arr=np.column_stack((t,iter_interp,cost_interp))

                    tmp = pd.DataFrame(arr, columns=["time", "iteration","cost"])
                    tmp['query']=actual_query_number
                    tmp['repetition']=rep
                    tmp['pipeline']=pipeline_id
                    tmp['planner']=planner_id
                    tmp['scenario']=scenario
                    tmp['group_name']=group_name
                    tmp['test_name']=test_name

                    if first_run:
                        res = tmp
                        first_run=False
                    else:
                        res=res.append(tmp, ignore_index=True)


                    res.to_csv(test_name+"_res.csv")

        for og in object_groups:
            uog_req=object_loader_msgs.srv.RemoveObjectsGroupRequest()
            uog_req.objects_group=og
            obj_unloader_srv(uog_req)

if __name__ == '__main__':
    runQuery()
