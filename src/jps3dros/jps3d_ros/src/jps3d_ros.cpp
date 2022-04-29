/* ----------------------------------------------------------------------------
 * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#include "jps3d_ros.hpp"
#include <sensor_msgs/point_cloud_conversion.h>

// This object is created in the jps3d_cvx_ros_node
jps3d_ROS::jps3d_ROS(ros::NodeHandle nh) : nh_(nh)
{

  safeGetParam(nh_, "dc", par_.dc);
  safeGetParam(nh_, "goal_radius", par_.goal_radius);
  safeGetParam(nh_, "drone_radius", par_.drone_radius);
  safeGetParam(nh_, "force_goal_height", par_.force_goal_height);
  safeGetParam(nh_, "goal_height", par_.goal_height);


  safeGetParam(nh_, "Ra", par_.Ra);

  safeGetParam(nh_, "z_ground", par_.z_ground);
  //safeGetParam(nh_, "z_max", par_.z_max);
  safeGetParam(nh_, "inflation_jps", par_.inflation_jps);
  safeGetParam(nh_, "factor_jps", par_.factor_jps);
  safeGetParam(nh_, "max_poly_whole", par_.max_poly_whole);

  safeGetParam(nh_, "dist_max_vertexes", par_.dist_max_vertexes);

  // And now obtain the parameters from the mapper
  std::vector<double> world_dimensions;
  safeGetParam(nh_, "world_dimensions", world_dimensions);
  safeGetParam(nh_, "resolution", par_.res);
  
  safeGetParam(nh_, "setup/jps_3d_on_goal_loc", par_.pub_goal_set_name);

  par_.wdx = world_dimensions[0];
  par_.wdy = world_dimensions[1];
  par_.wdz = world_dimensions[2];

  std::cout << bold << green << "world_dimensions=" << world_dimensions << reset << std::endl;
  std::cout << bold << green << "resolution=" << par_.res << reset << std::endl;
  std::cout << "Parameters obtained" << std::endl;

  if (par_.factor_jps * par_.res / 2.0 > par_.inflation_jps)
  {
    std::cout << bold << red << "Needed: par_.factor_jps * par_.res / 2 <= par_.inflation_jps" << reset
              << std::endl;  // If not JPS will find a solution between the voxels.
    abort();
  }

  // Initialize jps3d
  jps3d_ptr_ = std::unique_ptr<jps3d>(new jps3d(par_));
  ROS_INFO("Planner initialized");

  // Publishers
  pub_global_plan = nh_.advertise<nav_msgs::Path>("global_plan", 1);
  pub_jps_time = nh_.advertise<std_msgs::Float64>("jps_time", 1);
  // Subscribers
  occup_grid_sub_.subscribe(nh_, "/occup_grid", 1);
  unknown_grid_sub_.subscribe(nh_, "/unknown_grid", 1);
  sync_.reset(new Sync(MySyncPolicy(1), occup_grid_sub_, unknown_grid_sub_));
  sync_->registerCallback(boost::bind(&jps3d_ROS::mapCB, this, _1, _2));
  sub_goal_ = nh_.subscribe("/move_base_simple/goal", 1, &jps3d_ROS::terminalGoalCB, this);
    if (par_.pub_goal_set_name == true)
  {
    sub_goal_ = nh_.subscribe("/goal_loc", 1, &jps3d_ROS::terminalGoalCB, this);
  }

  sub_state_ = nh_.subscribe("state", 1, &jps3d_ROS::stateCB, this);
  // Timers
  replanCBTimer_ = nh_.createTimer(ros::Duration(par_.dc), &jps3d_ROS::replanCB, this);
}

void jps3d_ROS::replanCB(const ros::TimerEvent& e){
  if (ros::ok())
  {
    vec_Vecf<3> JPS_soln;
    float jps_time;

    jps3d_ptr_->replan(JPS_soln,jps_time);
    publishJPSPath(JPS_soln,jps_time); //Vandan Added this
    
  }
}
void jps3d_ROS::stateCB(const snapstack_msgs::State& msg){
  state state_tmp;
  state_tmp.setPos(msg.pos.x, msg.pos.y, msg.pos.z);
  state_tmp.setVel(msg.vel.x, msg.vel.y, msg.vel.z);
  state_tmp.setAccel(0.0, 0.0, 0.0);
  double roll, pitch, yaw;
  quaternion2Euler(msg.quat, roll, pitch, yaw);
  state_tmp.setYaw(yaw);
  jps3d_ptr_->updateState(state_tmp);
}
void jps3d_ROS::publishJPSPath(vec_Vecf<3>& path,float& jps_time){
  nav_msgs::Path globalplan;
  geometry_msgs::PoseStamped pose;
  
  std_msgs::Float64 jps_time_;
  jps_time_.data = jps_time;

  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = "vicon";
  globalplan.header.stamp =  ros::Time::now();
  globalplan.header.frame_id = "vicon";
  for (const auto& it : path)
  {
    geometry_msgs::Point p = eigen2point(it);
    pose.pose.position = p;
    globalplan.poses.push_back(pose);
  }
  pub_global_plan.publish(globalplan);
  pub_jps_time.publish(jps_time_);
}
// Occupied CB
void jps3d_ROS::mapCB(const sensor_msgs::PointCloud2::ConstPtr& pcl2ptr_map_ros,
                      const sensor_msgs::PointCloud2::ConstPtr& pcl2ptr_unk_ros){
  // Occupied Space Point Cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclptr_map(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*pcl2ptr_map_ros, *pclptr_map);
  // Unknown Space Point Cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclptr_unk(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*pcl2ptr_unk_ros, *pclptr_unk);

  jps3d_ptr_->updateMap(pclptr_map, pclptr_unk);
}
void jps3d_ROS::terminalGoalCB(const geometry_msgs::PoseStamped& msg){
  state G_term;
  double height;
  if (par_.force_goal_height)
  {
    height = par_.goal_height;
  }
  else
  {
    height = msg.pose.position.z;
  }
  G_term.setPos(msg.pose.position.x, msg.pose.position.y, height);
  jps3d_ptr_->setTerminalGoal(G_term);
  state G;  // projected goal
  jps3d_ptr_->getG(G);
}
