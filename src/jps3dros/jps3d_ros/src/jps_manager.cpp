/* ----------------------------------------------------------------------------
 * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#include "jps_manager.hpp"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Path.h"
#include "visualization_msgs/MarkerArray.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <Eigen/StdVector>

#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <stdio.h>
#include <math.h>
#include <algorithm>
#include <vector>
#include <assert.h>
#include <stdlib.h>

#include <sensor_msgs/point_cloud_conversion.h>

#include "termcolor.hpp"

// using namespace JPS;

JPS_Manager::JPS_Manager()
{
  map_util_ = std::make_shared<JPS::VoxelMapUtil>();
  planner_ptr_ = std::unique_ptr<JPSPlanner3D>(new JPSPlanner3D(false));
}

void JPS_Manager::setNumCells(int cells_x, int cells_y, int cells_z)
{
  cells_x_ = cells_x;
  cells_y_ = cells_y;
  cells_z_ = cells_z;
}

void JPS_Manager::setFactorJPS(double factor_jps)
{
  factor_jps_ = factor_jps;
}

void JPS_Manager::setResolution(double res)
{
  res_ = res;
}

void JPS_Manager::setInflationJPS(double inflation_jps)
{
  inflation_jps_ = inflation_jps;
}

void JPS_Manager::setZGroundAndZMax(double z_ground, double z_max)
{
  z_ground_ = z_ground;
  z_max_ = z_max;
}

void JPS_Manager::setVisual(bool visual)
{
  visual_ = visual;
}

void JPS_Manager::setDroneRadius(double drone_radius)
{
  drone_radius_ = drone_radius;
}

void JPS_Manager::updateJPSMap(pcl::PointCloud<pcl::PointXYZ>::Ptr pclptr, Eigen::Vector3d& center)
{
  Vec3f center_map = center;  // state_.pos;

  mtx_jps_map_util.lock();

  map_util_->readMap(pclptr, cells_x_, cells_y_, cells_z_, factor_jps_ * res_, center_map, z_ground_, z_max_,
                     inflation_jps_);  // Map read

  mtx_jps_map_util.unlock();
}

vec_Vecf<3> JPS_Manager::solveJPS3D(Vec3f& start_sent, Vec3f& goal_sent, bool* solved, int i)
{
  Eigen::Vector3d start(start_sent(0), start_sent(1), std::max(start_sent(2), 0.0));
  Eigen::Vector3d goal(goal_sent(0), goal_sent(1), std::max(goal_sent(2), 0.0));

  Vec3f originalStart = start;

  pcl::PointXYZ pcl_start = eigenPoint2pclPoint(start);
  pcl::PointXYZ pcl_goal = eigenPoint2pclPoint(goal);

  ///////////////////////////////////////////////////////////////
  /////////////////////////// RUN JPS ///////////////////////////
  ///////////////////////////////////////////////////////////////

  mtx_jps_map_util.lock();

  // Set start and goal free
  const Veci<3> start_int = map_util_->floatToInt(start);
  const Veci<3> goal_int = map_util_->floatToInt(goal);

  map_util_->setFreeVoxelAndSurroundings(start_int, inflation_jps_);
  map_util_->setFreeVoxelAndSurroundings(goal_int, inflation_jps_);

  planner_ptr_->setMapUtil(map_util_);  // Set collision checking function

  bool valid_jps = planner_ptr_->plan(start, goal, 1, true);  // Plan from start to goal with heuristic weight=1, and
                                                              // using JPS (if false --> use A*)

  vec_Vecf<3> path;
  path.clear();

  if (valid_jps == true)  // There is a solution
  {
    path = planner_ptr_->getPath();  // getpar_.RawPath() if you want the path with more corners (not "cleaned")
    if (path.size() > 1)
    {
      path[0] = start;
      path[path.size() - 1] = goal;  // force to start and end in the start and goal (and not somewhere in the voxel)
    }
    else
    {  // happens when start and goal are very near (--> same cell)
      vec_Vecf<3> tmp;
      tmp.push_back(start);
      tmp.push_back(goal);
      path = tmp;
    }
  }
  else
  {
    std::cout << "JPS didn't find a solution from" << start.transpose() << " to " << goal.transpose() << std::endl;
  }
  mtx_jps_map_util.unlock();

  *solved = valid_jps;
  return path;
}