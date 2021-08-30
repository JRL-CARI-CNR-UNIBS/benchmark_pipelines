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
#include <moveit_planning_helper/manage_trajectories.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "generate_queries");
  ros::NodeHandle pnh("~");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(4);
  spinner.start();


  std::string group_name="manipulator";
  if (!pnh.getParam("group_name",group_name))
  {
    ROS_ERROR("%s/group_name not defined",pnh.getNamespace().c_str());
    return 0;
  }


  int starting_query=0;
  if (!pnh.getParam("starting_query",starting_query))
  {
    ROS_INFO("%s/starting_query not defined, using 0",pnh.getNamespace().c_str());
    starting_query= 0;
  }

  int queries_number=100;
  if (!pnh.getParam("queries_number",queries_number))
  {
    ROS_ERROR("%s/queries_number not defined",pnh.getNamespace().c_str());
    return 0;
  }

  std::string query_prefix;
  if (!pnh.getParam("query_prefix",query_prefix))
  {
    ROS_ERROR("%s/query_prefix not defined",pnh.getNamespace().c_str());
    return 0;
  }



  moveit::planning_interface::MoveGroupInterface move_group(group_name);
  move_group.setPlanningTime(1);

  std::shared_ptr<tf2_ros::Buffer> tf_buffer = std::make_shared<tf2_ros::Buffer>();
  std::shared_ptr<tf2_ros::TransformListener> tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, nh);

  robot_model_loader::RobotModelLoaderPtr robot_model_loader=std::make_shared<robot_model_loader::RobotModelLoader>("robot_description");
  robot_model::RobotModelPtr robot_model = robot_model_loader->getModel();


  ros::ServiceClient ps_client=nh.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");

  moveit_msgs::GetPlanningScene srv;
  ps_client.call(srv);


  planning_scene::PlanningScene scene(robot_model);

  scene.setPlanningSceneMsg(srv.response.scene);
  moveit_msgs::PlanningScene scene_msg;
  scene.getPlanningSceneMsg(scene_msg);
  moveit::core::RobotState state(*move_group.getCurrentState());
  std::vector<double> configuration;
  state.copyJointGroupPositions(group_name,configuration);
  query_prefix+="_dof_"+std::to_string(configuration.size());
  pnh.setParam("query_test",query_prefix);

  int actual_query_number=starting_query;
  while (actual_query_number<queries_number)
  {
    state.setToRandomPositions();
    state.update();
    if (!scene.isStateValid(state,group_name))
    {
      continue;
    }
    std::vector<double> start_configuration;
    std::vector<double> goal_configuration;
    state.copyJointGroupPositions(group_name,start_configuration);
    move_group.setStartState(state);

    state.setToRandomPositions();
    state.update();
    if (!scene.isStateValid(state,group_name))
    {
      continue;
    }
    state.copyJointGroupPositions(group_name,goal_configuration);
    move_group.setJointValueTarget(state);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::planning_interface::MoveItErrorCode plan_exit_code = move_group.plan(plan);
    if (not plan_exit_code)
    {
      continue;
    }
    double length=trajectory_processing::computeTrajectoryLength(plan.trajectory_.joint_trajectory);
    double utopia=0;
    for (size_t idx=0;idx<start_configuration.size();idx++)
      utopia+=std::pow(start_configuration.at(idx)-goal_configuration.at(idx),2);
    utopia=std::sqrt(utopia);
    if (length<1.0001*utopia)
    {
      ROS_DEBUG_STREAM("directed connection,skip");
      continue;
    }
    std::string query_name=query_prefix;
    pnh.setParam(query_prefix+"/query_"+std::to_string(actual_query_number)+"/start_configuration",start_configuration);
    pnh.setParam(query_prefix+"/query_"+std::to_string(actual_query_number)+"/goal_configuration" ,goal_configuration);
    actual_query_number++;
  }
  pnh.setParam(query_prefix+"/queries_number",actual_query_number);


  return 0;
}
