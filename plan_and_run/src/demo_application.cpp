/*
 * demo_application.cpp
 *
 *  Created on: Apr 9, 2015
 *      Author: ros-devel
 */

#include <plan_and_run/demo_application.h>

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
      nh.getParam("controller_joint_names",config_.joint_names) &&
      ph.getParam("trajectory/time_delay",config_.time_delay) &&
      ph.getParam("trajectory/foci_distance",config_.foci_distance) &&
      ph.getParam("trajectory/radius",config_.radius) &&
      ph.getParam("trajectory/num_points",config_.num_points) &&
      ph.getParam("trajectory/num_lemniscates",config_.num_lemniscates) &&
      ph.getParam("trajectory/center",config_.center))
  {
    ROS_INFO_STREAM("Loaded application parameters");
  }
  else
  {
    ROS_ERROR_STREAM("Failed to load application parameters");
    exit(-1);
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
    exit(-1);
  }

  // Planner initialization
  if(planner_.initialize(robot_model_ptr_))
  {
    ROS_INFO_STREAM("Descartes Dense Planner initialized");
  }
  else
  {
    ROS_ERROR_STREAM("Failed to initialize Dense Planner");
    exit(-1);
  }

}

void DemoApplication::initMoveitClient()
{
  ros::NodeHandle nh;

  moveit_run_path_client_ = nh.serviceClient<moveit_msgs::ExecuteKnownTrajectory>(EXECUTE_TRAJECTORY_SERVICE,true);
  if(moveit_run_path_client_.waitForExistence(ros::Duration(SERVICE_TIMEOUT)))
  {
    ROS_INFO_STREAM("Connected to '"<<moveit_run_path_client_.getService()<<"' service");
  }
  else
  {
    ROS_ERROR_STREAM("Failed to connect to '"<< moveit_run_path_client_.getService()<<"' service");
    exit(-1);
  }
}

void DemoApplication::generateTrajectory(DescartesTrajectory& traj)
{
  double a = config_.foci_distance;
  double ro = config_.radius;
  int npoints = config_.num_points;
  int nlemns = config_.num_lemniscates;
  Eigen::Vector3d offset(config_.center[0],config_.center[1],config_.center[2]);
  Eigen::Vector3d unit_z,unit_y,unit_x;

  // generating polar angle values
  std::vector<double> theta(npoints);

  // interval 1 <-pi/4 , pi/4 >
  double d_theta = 2*M_PI_2/(npoints - 1);
  for(unsigned int i = 0; i < npoints/2;i++)
  {
    theta[i] = -M_PI_4  + i * d_theta;
  }
  theta[0] = theta[0] + EPSILON;
  theta[npoints/2 - 1] = theta[npoints/2 - 1] - EPSILON;

  // interval 2 < 3*pi/4 , 5 * pi/4 >
  for(unsigned int i = 0; i < npoints/2;i++)
  {
    theta[npoints/2 + i] = 3*M_PI_4  + i * d_theta;
  }
  theta[npoints/2] = theta[npoints/2] + EPSILON;
  theta[npoints - 1] = theta[npoints - 1] - EPSILON;

  // generating omega angle (lemniscate angle offset)
  std::vector<double> omega(nlemns);
  double d_omega = M_PI/(nlemns - 1);
  for(unsigned int i = 0; i < nlemns;i++)
  {
     omega[i] = i*d_omega;
  }

  Eigen::Affine3d pose;
  double x,y,z,r,phi;

  traj.clear();
  traj.reserve(nlemns*npoints);
  for(unsigned int j = 0; j < nlemns;j++)
  {
    for(unsigned int i = 0 ; i < npoints;i++)
    {
      r = std::sqrt( std::pow(a,2) * std::cos(2*theta[i]) );
      phi = r < ro ? std::asin(r/ro):  (M_PI - std::asin((2*ro - r)/ro) );

      x = ro * std::cos(theta[i] + omega[j]) * std::sin(phi);
      y = ro * std::sin(theta[i] + omega[j]) * std::sin(phi);
      z = ro * std::cos(phi);

      // determining orientation
      unit_z <<x, y , z;
      unit_z.normalize();

      unit_x = (Eigen::Vector3d(0,1,0).cross( unit_z)).normalized();
      unit_y = (unit_z .cross(unit_x)).normalized();

      Eigen::Matrix3d rot;
      rot << unit_x(0),unit_y(0),unit_z(0)
         ,unit_x(1),unit_y(1),unit_z(1)
         ,unit_x(2),unit_y(2),unit_z(2);

      pose = Eigen::Translation3d(offset(0) + x,
                                  offset(1) + y,
                                  offset(2) + z) * rot;

      traj.push_back(makeCartesianPoint(pose));
    }
  }

}

descartes_core::TrajectoryPtPtr DemoApplication::makeCartesianPoint(const Eigen::Affine3d& pose)
{
  using namespace descartes_core;
  using namespace descartes_trajectory;

  return TrajectoryPtPtr( new AxialSymmetricPt(pose,0.1f,AxialSymmetricPt::FreeAxis::Z_AXIS) );
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
    exit(-1);
  }

  // retrieving robot path
  if(!planner_.getPath(output_path) || output_path.empty())
  {
    ROS_ERROR_STREAM("Failed to retrieve robot path");
  }
}

void DemoApplication::runPath(const DescartesTrajectory& path)
{

  // creating move group to move the arm in free space
  move_group_interface::MoveGroup move_group(config_.group_name);

  // creating goal joint pose to start of the path
  std::vector<double> seed_pose(robot_model_ptr_->getDOF()),start_pose;
  path[0]->getNominalJointPose(seed_pose,*robot_model_ptr_,start_pose);

  // moving arm to joint goal
  move_group.setJointValueTarget(start_pose);
  moveit_msgs::MoveItErrorCodes result = move_group.move();
  if(result.val != result.SUCCESS)
  {
    ROS_ERROR_STREAM("Move to start joint pose failed");
    exit(-1);
  }

  // sending path to robot
  moveit_msgs::RobotTrajectory moveit_traj;
  fromDescartesToMoveitTrajectory(path,moveit_traj.joint_trajectory);
  moveit_msgs::ExecuteKnownTrajectory srv;
  srv.request.trajectory = moveit_traj;
  srv.request.wait_for_execution = true;
  if(moveit_run_path_client_.call(srv))
  {
    ROS_INFO_STREAM("Robot path execution completed");
  }
  else
  {
    ROS_ERROR_STREAM("Failed to run robot path with error "<<srv.response.error_code.val);
    exit(-1);
  }

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
    joint_point->getNominalJointPose(std::vector<double>(), *robot_model_ptr_, joints);

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
