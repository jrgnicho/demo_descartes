/*
 * demo_application.h
 *
 *  Created on: Apr 9, 2015
 *      Author: ros-devel
 */

#ifndef DEMO_DESCARTES_PLAN_AND_RUN_INCLUDE_PLAN_AND_RUN_DEMO_APPLICATION_H_
#define DEMO_DESCARTES_PLAN_AND_RUN_INCLUDE_PLAN_AND_RUN_DEMO_APPLICATION_H_

#include <ros/ros.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <moveit/move_group_interface/move_group.h>
#include <descartes_moveit/moveit_state_adapter.h>
#include <descartes_trajectory/axial_symmetric_pt.h>
#include <descartes_trajectory/cart_trajectory_pt.h>
#include <descartes_planner/dense_planner.h>

namespace plan_and_run
{

const std::string ROBOT_DESCRIPTION_PARAM = "robot_description";
const std::string EXECUTE_TRAJECTORY_SERVICE = "/execute_kinematic_path";
const double SERVICE_TIMEOUT = 5.0f; // seconds
const double EPSILON = 0.0001f;

typedef std::vector<descartes_core::TrajectoryPtPtr> DescartesTrajectory;

struct DemoConfiguration
{
  std::string group_name;
  std::string tip_link;
  std::string base_link;
  std::string world_frame;
  std::vector<std::string> joint_names;

  // trajectory generation
  double time_delay;
  double foci_distance;
  double radius;
  int num_points;
  int num_lemniscates;
  std::vector<double> center;
};

class DemoApplication
{
public:
  DemoApplication();
  virtual ~DemoApplication();

  void loadParameters();
  void initMoveitClient();
  void initDescartes();
  void generateTrajectory(DescartesTrajectory& traj);
  void planPath(const DescartesTrajectory& input_traj,DescartesTrajectory& output_path);
  void runPath(const DescartesTrajectory& path);


protected:

  void fromDescartesToMoveitTrajectory(const DescartesTrajectory& in_traj,
                                              trajectory_msgs::JointTrajectory& out_traj);
  static descartes_core::TrajectoryPtPtr makeCartesianPoint(const Eigen::Affine3d& pose);

protected:

  DemoConfiguration config_;
  ros::ServiceClient moveit_run_path_client_;

  descartes_core::RobotModelPtr robot_model_ptr_;
  descartes_planner::DensePlanner planner_;

};

} /* namespace plan_and_run */

#endif /* DEMO_DESCARTES_PLAN_AND_RUN_INCLUDE_PLAN_AND_RUN_DEMO_APPLICATION_H_ */
