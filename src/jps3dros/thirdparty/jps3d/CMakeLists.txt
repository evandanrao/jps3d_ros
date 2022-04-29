cmake_minimum_required(VERSION 2.8)
project(jps3d)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wno-deprecated-declarations")

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE RelWithDebInfo)
ENDIF()

set(BUILD_SHARED_LIBS ON)


find_package(catkin REQUIRED COMPONENTS 
  roscpp
  pcl_ros
)

FIND_PACKAGE(roscpp  REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(PkgConfig REQUIRED)
PKG_CHECK_MODULES(YAMLCPP REQUIRED yaml-cpp)

include_directories(${catkin_INCLUDE_DIRS})

IF(YAMLCPP_FOUND)
  MESSAGE(STATUS "Found yaml-cpp.")
ELSE()
  MESSAGE(FATAL_ERROR "Could not find yaml-cpp.")
ENDIF()

INCLUDE_DIRECTORIES(include ${EIGEN3_INCLUDE_DIRS} ${PCL_LIBRARY_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(jps_lib src/jps_planner/graph_search.cpp
  src/jps_planner/jps_planner.cpp ${PCL_LIBRARIES})

install(FILES "${PROJECT_NAME}Config.cmake" "${PROJECT_NAME}ConfigVersion.cmake"
  DESTINATION "share/${PROJECT_NAME}/cmake")

#install(TARGETS jps_lib dmp_lib
#  RUNTIME DESTINATION bin
#  LIBRARY DESTINATION lib
#  ARCHIVE DESTINATION lib)


install(TARGETS jps_lib
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(DIRECTORY include/jps_basis include/jps_planner include/jps_collision
  DESTINATION include)