/*
 * demo_application.cpp
 *
 *  Created on: Apr 9, 2015
 *      Author: ros-devel
 */

#include "demo_application.h"

namespace plan_and_run
{

DemoApplication::DemoApplication()
{
  // TODO Auto-generated constructor stub

}

DemoApplication::~DemoApplication()
{
  // TODO Auto-generated destructor stub
}

void DemoApplication::loadParameters()
{
  ros::NodeHandle ph("~");
  ros::NodeHandle nh;

  if(ph.getParam("group_name",config_.group_name) &&
      ph.getParam("tip_link",config_.tip_link) &&
      ph.getParam("base_link",config_.base_link) &&
      ph.getParam("world_frame",config_.world_frame) &&
      ph.getParam("time_delay",config_.time_delay) &&
      nh.getParam("controller_joint_names",config_.joint_names))
  {
    ROS_INFO_STREAM("Loaded application parameters");
  }
  else
  {
    ROS_ERROR_STREAM("Failed to load application parameters");
    exit();
  }

}

void DemoApplication::initDescartes()
{
  // Robot Model initialization
  robot_model_ptr_.reset(new descartes_moveit::MoveitStateAdapter());
  if(robot_model_ptr_->initialize(ROBOT_DESCRIPTION_PARAM,
                                  config_.group_name,
                                  config_.base_link,
                                  config_.tip_link))
  {
    ROS_INFO_STREAM("Descartes Robot Model initialized");
  }
  else
  {
    ROS_ERROR_STREAM("Failed to initialize Robot Model");
    exit();
  }

  // Planner initialization
  if(planner_.initialize(robot_model_ptr_))
  {
    ROS_INFO_STREAM("Descartes Dense Planner initialized");
  }
  else
  {
    ROS_ERROR_STREAM("Failed to initialize Dense Planner");
    exit();
  }

}

void DemoApplication::initMoveitClient()
{
  ros::NodeHandle nh;

  moveit_run_path_client_ = nh.serviceClient<moveit_msgs::ExecuteKnownTrajectory>(EXECUTE_TRAJECTORY_SERVICE,true);
  if(moveit_run_path_client_.waitForExistence(SERVICE_TIMEOUT))
  {
    ROS_INFO_STREAM("Connected to '"<<moveit_run_path_client_.getService()<<"' service");
  }
  else
  {
    ROS_ERROR_STREAM("Failed to connect to '"<< moveit_run_path_client_.getService()<<"' service");
    exit();
  }
}

void DemoApplication::generateTrajectory(DescartesTrajectory& traj)
{

}

void DemoApplication::planPath(const DescartesTrajectory& input_traj,DescartesTrajectory& output_path)
{

  // planning robot path
  if (planner_.planPath(input_traj))
  {
    ROS_INFO_STREAM("Valid path was found");
  }
  else
  {
    ROS_ERROR_STREAM("Could not solve for a valid path");
    exit();
  }

  // retrieving robot path
  if(!planner_.getPath(output_path) || output_path.empty())
  {
    ROS_ERROR_STREAM("Failed to retrieve robot path");
  }
}

void DemoApplication::runPath(const DescartesTrajectory& path)
{


}

void DemoApplication::fromDescartesToMoveitTrajectory(const DescartesTrajectory& in_traj,
                                                      trajectory_msgs::JointTrajectory& out_traj)
{
  // Fill out information about our trajectory
  out_traj.header.stamp = ros::Time::now();
  out_traj.header.frame_id = config_.world_frame;
  out_traj.joint_names = config_.joint_names;

  // For keeping track of time-so-far in the trajectory
  double time_offset = 0.0;

  // Loop through the trajectory
  for (unsigned int i = 0; i < in_traj.size(); i++)
  {
    // Find nominal joint solution at this point
    std::vector<double> joints;

    // getting joint position at current point
    const descartes_core::TrajectoryPtPtr& joint_point = in_traj[i];
    joint_point->getNominalJointPose(std::vector<double>(), model, joints);

    // Fill out a ROS trajectory point
    trajectory_msgs::JointTrajectoryPoint pt;
    pt.positions = joints;
    // velocity, acceleration, and effort are given dummy values
    // we'll let the controller figure them out
    pt.velocities.resize(joints.size(), 0.0);
    pt.accelerations.resize(joints.size(), 0.0);
    pt.effort.resize(joints.size(), 0.0);
    // set the time into the trajectory
    pt.time_from_start = ros::Duration(time_offset);
    // increment time
    time_offset += config_.time_delay;

    out_traj.points.push_back(pt);
  }

}

} /* namespace plan_and_run */
