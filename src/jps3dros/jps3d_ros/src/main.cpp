/* ----------------------------------------------------------------------------
 * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- 
 * Converted the original faster code to extract only the JPS3D ROS Wrapper out to be usable as a standalone package
 * Author : Vandan Eddya Rao >  * To be used for personal use only*/
#include "jps3d_ros.hpp"

int main(int argc, char **argv)
{
  // Initializes ROS, and sets up a node
  ros::init(argc, argv, "jps3d");
  ros::NodeHandle nh("~");
  ros::CallbackQueue custom_queue1;
  nh.setCallbackQueue(&custom_queue1);

  jps3d_ROS jps3d_ROS(nh);

  ros::AsyncSpinner spinner1(0, &custom_queue1);
  spinner1.start();  // start spinner of the custom queue 1
  ros::spin();  // spin the normal queue
  ros::waitForShutdown();
  return 0;
}