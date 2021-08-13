/*
Copyright (c) 2020, JRL-CARI CNR-STIIMA/UNIBS
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
*/


#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <ros/ros.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene_monitor/current_state_monitor.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <moveit/kinematic_constraints/utils.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <rosparam_utilities/rosparam_utilities.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <std_msgs/String.h>
#include <moveit_planning_helper/manage_trajectories.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "generate_queries");
  ros::NodeHandle pnh("~");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(4);
  spinner.start();


  ros::Publisher planner_pub=pnh.advertise<std_msgs::String>("planner_id",1);
  std_msgs::String planner_id_msg;

  std::string group_name="manipulator";
  if (!pnh.getParam("group_name",group_name))
  {
    ROS_ERROR("%s/group_name not defined",pnh.getNamespace().c_str());
    return 0;
  }


  int repetitions=100;
  if (!pnh.getParam("repetitions",repetitions))
  {
    ROS_ERROR("%s/repetitions not defined",pnh.getNamespace().c_str());
    return 0;
  }

  std::string query_prefix;
  if (!pnh.getParam("query_prefix",query_prefix))
  {
    ROS_ERROR("%s/query_prefix not defined",pnh.getNamespace().c_str());
    return 0;
  }

  int queries_number=100;
  if (!pnh.getParam(query_prefix+"/queries_number",queries_number))
  {
    ROS_ERROR("%s/queries_number not defined",pnh.getNamespace().c_str());
    return 0;
  }

  std::vector<double> planning_times;
  if (!pnh.getParam("planning_times",planning_times))
  {
    ROS_ERROR("%s/planning_times not defined",pnh.getNamespace().c_str());
    return 0;
  }

  std::vector<std::string> pipeline_ids;
  if (!pnh.getParam("pipeline_ids",pipeline_ids))
  {
    ROS_ERROR("%s/pipeline_ids not defined",pnh.getNamespace().c_str());
    return 0;
  }

  std::map<std::string,std::vector<std::string>> planner_ids;
  for (const std::string& pipeline_id: pipeline_ids)
  {
    std::vector<std::string> tmp;
    if (!pnh.getParam("planner_ids/"+pipeline_id,tmp))
    {
      ROS_ERROR("%s/planners_ids/%s not defined",pnh.getNamespace().c_str(),pipeline_id.c_str());
      return 0;
    }
    planner_ids.insert(std::pair<std::string,std::vector<std::string>>(pipeline_id,tmp));
  }


  moveit::planning_interface::MoveGroupInterface move_group(group_name);
  std::shared_ptr<tf2_ros::Buffer> tf_buffer = std::make_shared<tf2_ros::Buffer>();
  std::shared_ptr<tf2_ros::TransformListener> tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, nh);

  robot_model_loader::RobotModelLoaderPtr robot_model_loader=std::make_shared<robot_model_loader::RobotModelLoader>("robot_description");

  ros::Duration(1).sleep();
  move_group.startStateMonitor();


  moveit::core::RobotState state(*move_group.getCurrentState());


  for (const double& planning_time: planning_times)
  {

    move_group.setPlanningTime(planning_time);
    for (const std::string& pipeline_id: pipeline_ids)
    {
      for (const std::string& planner_id: planner_ids.at(pipeline_id))
      {

        move_group.setPlanningPipelineId(pipeline_id);
        move_group.setPlannerId(planner_id);

        ROS_INFO("PIPELINE = %s, PLANNER = %s. Planning time = %f",pipeline_id.c_str(),planner_id.c_str(),planning_time);

        int actual_query_number=0;
        while (actual_query_number<queries_number)
        {
          ROS_DEBUG("PIPELINE = %s, PLANNER = %s. query %d of %d",pipeline_id.c_str(),planner_id.c_str(),actual_query_number+1,queries_number);
          planner_id_msg.data=pipeline_id+"/"+planner_id+" query "+std::to_string(actual_query_number);
          planner_pub.publish(planner_id_msg);

          std::string query_name=query_prefix;
          std::vector<double> start_configuration;
          std::vector<double> goal_configuration;

          pnh.getParam(query_prefix+"/query_"+std::to_string(actual_query_number)+"/start_configuration",start_configuration);
          pnh.getParam(query_prefix+"/query_"+std::to_string(actual_query_number)+"/goal_configuration" ,goal_configuration);


          state.setJointGroupPositions(group_name,start_configuration);
          state.update();
          move_group.setStartState(state);

          state.setJointGroupPositions(group_name,goal_configuration);
          state.update();
          move_group.setJointValueTarget(state);

          moveit::planning_interface::MoveGroupInterface::Plan plan;
          for (int repetition=0;repetition<repetitions;repetition++)
          {
            moveit::planning_interface::MoveItErrorCode plan_exit_code = move_group.plan(plan);


            std::string result_prefix=query_prefix+"/query_"+std::to_string(actual_query_number)+"/"+pipeline_id+"/"+planner_id+"/iteration_"+std::to_string(repetition)+"/planning_time_ms_"+std::to_string((int)(1000.0*planning_time));

            pnh.setParam(result_prefix+"/planning_time",plan.planning_time_);
            pnh.setParam(result_prefix+"/error_code",plan_exit_code.val);
            if (plan_exit_code)
            {
              pnh.setParam(result_prefix+"/trajectory_time",plan.trajectory_.joint_trajectory.points.back().time_from_start.toSec());
              double length=trajectory_processing::computeTrajectoryLength(plan.trajectory_.joint_trajectory);
              pnh.setParam(result_prefix+"/trajectory_length",length);
            }
          }
          actual_query_number++;
        }
      }
    }
  }

  return 0;
}
