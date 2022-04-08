#!/usr/bin/env python3
# license removed for brevity
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


performance=np.array([])
received=False;

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

    group_name=rospy.get_param("~group_name")
    repetitions=rospy.get_param("~repetitions")
    query_test=rospy.get_param("~query_test")
    queries_number=rospy.get_param("~queries_number")
    starting_query=rospy.get_param("~starting_query")
    planning_times=rospy.get_param("~planning_times")
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


    first_run=True
    planning_time=max(planning_times)
    move_group.set_planning_time(planning_time)


    for actual_query_number in range(starting_query,queries_number):
        print("query: ",actual_query_number)

        start_configuration = rospy.get_param("~"+query_test+"/query_"+str(actual_query_number)+"/start_configuration")
        goal_configuration  = rospy.get_param("~"+query_test+"/query_"+str(actual_query_number)+"/goal_configuration")

        state.joint_state.position=start_configuration
        move_group.set_start_state(state)
        move_group.set_joint_value_target(goal_configuration)

        for pp in planner_ids:
            pipeline_id=pp[0]
            planner_id=pp[1]
            print(pipeline_id,", ",planner_id)
            move_group.set_planning_pipeline_id(pipeline_id)
            move_group.set_planner_id(planner_id)


            for rep in range(repetitions):
                print("repetition = ",rep)
                received=False
                result=move_group.plan()
                while (not received):
                    rospy.sleep(0.001)

                time=array[:,0]
                itera=array[:,1]
                cost=array[:,2]
                t=np.arange(0, planning_time, 0.01)
                cost_interp = np.interp(t, time, cost)
                iter_interp = np.round_(np.interp(t, time, itera))
                arr=np.column_stack((t,iter_interp,cost_interp))


                tmp = pd.DataFrame(arr, columns=["time", "iteration","cost"])
                tmp['query']=actual_query_number
                tmp['repetition']=rep
                tmp['pipeline']=pipeline_id
                tmp['planner']=planner_id
                tmp['test_name']=query_test
                if first_run:
                    res = tmp
                    first_run=False
                else:
                    res=res.append(tmp, ignore_index=True)


                res.to_csv("res.csv")

if __name__ == '__main__':
    runQuery()
