cmake_minimum_required(VERSION 3.0.2)
project(kimm_path_planner_ros_interface)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17 -Ofast)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)

find_package(kimm_trajectory_smoother REQUIRED)


## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Obstacle2D.msg
  MobileTrajectory.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  plan_mobile_path.srv
  action_mobile_path.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS roscpp std_msgs geometry_msgs mzcommon steering_functions kimm_path_planner kimm_trajectory_smoother
 DEPENDS system_lib
)

find_package(kimm_path_planner REQUIRED)
find_package(ompl REQUIRED)
find_package(mzcommon REQUIRED)
find_package(chomp REQUIRED)
find_package(steering_functions REQUIRED)
find_package(kimm_trajectory_smoother REQUIRED)

link_directories(${Boost_LIBRARY_DIR})


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS}
  ${kimm_path_planner_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIR}
  ${mzcommon_INCLUDE_DIRS}
  ${kimm_trajectory_smoother_INCLUDE_DIRS}
)


#ros_tutorial_srv_server
add_executable(${PROJECT_NAME}_server src/path_planner_server.cpp)
target_link_libraries(${PROJECT_NAME}_server  ${catkin_LIBRARIES} kimm_path_planner kimm_trajectory_smoother)
add_dependencies(${PROJECT_NAME}_server kimm_path_planner_ros_interface_gencpp )

#ros_tutorial_srv_client
add_executable(${PROJECT_NAME}_client src/path_planner_client.cpp)
target_link_libraries(${PROJECT_NAME}_client   ${catkin_LIBRARIES} kimm_path_planner kimm_trajectory_smoother)
add_dependencies(${PROJECT_NAME}_client kimm_path_planner_ros_interface_gencpp )









