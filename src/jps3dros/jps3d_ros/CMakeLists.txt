cmake_minimum_required(VERSION 2.8.3)
project(jps3d_ros)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)
#add_definitions(-std=c99)

find_package( Eigen3 REQUIRED )
include_directories(${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS} include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}")

set(CMAKE_BUILD_TYPE Release) #From terminal: catkin config -DCMAKE_BUILD_TYPE=Release (or RelWithDebInfo)


find_package(catkin REQUIRED COMPONENTS 
	roscpp
	rospy
	snapstack_msgs
	sensor_msgs
	pcl_ros
)

find_package(jps3d REQUIRED)
include_directories(${JPS3D_INCLUDE_DIRS})

catkin_package(
#  INCLUDE_DIRS include
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(${catkin_INCLUDE_DIRS})
add_executable(${PROJECT_NAME}_node src/main.cpp src/jps3d.cpp src/jps3d_ros.cpp src/utils.cpp  src/jps_manager.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${JPS3D_LIBRARIES})
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS} )