cmake_minimum_required(VERSION 2.8.3)
project(dvrk_joint_publisher)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cisst_ros_bridge
  geometry_msgs
  roscpp
  roslib
  sensor_msgs
  std_msgs
  tf
)


set (REQUIRED_CISST_LIBRARIES
  cisstCommon
  cisstVector
  cisstVectorQt
  cisstOSAbstraction
  cisstMultiTask
  cisstParameterTypes
  cisstRobot
  cisstNumerical
  cisstQt
  )
  
find_package (cisst REQUIRED ${REQUIRED_CISST_LIBRARIES})
include (${CISST_USE_FILE})


# saw components 
cisst_find_saw_component (sawRobotIO1394 REQUIRED)
include_directories (${sawRobotIO1394_INCLUDE_DIR})
link_directories (${sawRobotIO1394_LIBRARY_DIR})

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  LIBRARIES dvrk_kinematics
  CATKIN_DEPENDS cisst_ros_bridge geometry_msgs roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# psm
add_executable(test_broadcaster src/test_broadcaster.cpp)
target_link_libraries (test_broadcaster ${sawRobotIO1394_LIBRARIES} ${catkin_LIBRARIES})
