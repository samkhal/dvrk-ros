cmake_minimum_required(VERSION 2.8.3)
project(dvrk_haptics)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  geometry_msgs
  roscpp
  roslib
  rospy
  sensor_msgs
  std_msgs
  tf
  tf_conversions
  ar_track_alvar

)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES dvrk_haptics
  CATKIN_DEPENDS std_msgs ar_track_alvar geometry_msgs
#  DEPENDS system_lib
)	

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

## Declare a cpp library
# add_library(dvrk_haptics
#   src/${PROJECT_NAME}/dvrk_haptics.cpp
# )

## Declare a cpp executable
 add_executable(Haptics_Main src/force_calc.cpp)
 #add_executable(Array_Parsing src/Tag_Parsing/tag_parsing.cpp)
 #add_executable(Data_Modifier src/Tag_Parsing/data_modifier.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(dvrk_haptics_node dvrk_haptics_generate_messages_cpp)

 target_link_libraries(Haptics_Main
   ${catkin_LIBRARIES}
 )

## Specify libraries to link a library or executable target against
 #target_link_libraries(Array_Parsing
  # ${catkin_LIBRARIES}
 #)

 #target_link_libraries(Data_Modifier
  # ${catkin_LIBRARIES}
 #)




