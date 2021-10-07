cmake_minimum_required(VERSION 2.8.3)

SET(PROJECT_NAME steering_functions)
SET(PROJECT_DESCRIPTION "kimm_trajectry with time opt ")
SET(PROJECT_URL "http://github.com/ggory15/kim_trajectory")

## Compile as C++11, supported in ROS Kinetic and newer
SET(CXX_DISABLE_WERROR TRUE)
SET(CMAKE_VERBOSE_MAKEFILE TRUE)
SET(PROJECT_USE_CMAKE_EXPORT TRUE)
add_compile_options(-std=c++11)

#INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/eigen.cmake)
INCLUDE(cmake/python.cmake)
INCLUDE(cmake/ide.cmake)
INCLUDE(cmake/apple.cmake)

find_package(catkin REQUIRED COMPONENTS)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 DEPENDS system_lib
)

include(FindPkgConfig)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)

find_package(ompl 1.5.0 REQUIRED)
find_package(ompl 1.5.0 REQUIRED)
include_directories(${OMPL_INCLUDE_DIRS})

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

## Get all source files
file(GLOB SRC_HC_CC_SS src/hc_cc_state_space/*.cpp)
## Get OMPL state space definitions
file(GLOB OMPL_SPACES include/ompl_state_spaces/*.hpp)

set(${PROJECT_NAME}_SOURCES
    src/dubins_state_space/dubins_state_space.cpp
    src/reeds_shepp_state_space/reeds_shepp_state_space.cpp
    src/utilities/utilities.cpp
    src/filter/ekf.cpp
    ${SRC_HC_CC_SS}
    ${OMPL_SPACES})

set(${PROJECT_NAME}_HEADERS
    include/steering_functions/steering_functions.hpp
    include/steering_functions/dubins_state_space/dubins_state_space.hpp
    include/steering_functions/filter/ekf.hpp
    include/steering_functions/hc_cc_state_space/cc00_dubins_state_space.hpp
    include/steering_functions/hc_cc_state_space/cc0pm_dubins_state_space.hpp
    include/steering_functions/hc_cc_state_space/cc00_reeds_shepp_state_space.hpp
    include/steering_functions/hc_cc_state_space/cc_dubins_state_space.hpp
    include/steering_functions/hc_cc_state_space/ccpm0_dubins_state_space.hpp
    include/steering_functions/hc_cc_state_space/ccpmpm_dubins_state_space.hpp
    include/steering_functions/hc_cc_state_space/configuration.hpp
    include/steering_functions/hc_cc_state_space/hc0pm_reeds_shepp_state_space.hpp
    include/steering_functions/hc_cc_state_space/hc00_reeds_shepp_state_space.hpp
    include/steering_functions/hc_cc_state_space/hc_cc_circle.hpp
    include/steering_functions/hc_cc_state_space/hc_cc_state_space.hpp
    include/steering_functions/hc_cc_state_space/hcpm0_reeds_shepp_state_space.hpp
    include/steering_functions/hc_cc_state_space/hcpmpm_reeds_shepp_state_space.hpp
    include/steering_functions/hc_cc_state_space/paths.hpp
    include/steering_functions/hc_cc_state_space/hc_reeds_shepp_state_space.hpp
    
    include/steering_functions/reeds_shepp_state_space/reeds_shepp_state_space.hpp
    include/steering_functions/utilities/utilities.hpp
    include/steering_functions/utilities/fresnel.data
    
    include/ompl_state_spaces/CurvatureStateSpace.hpp
    )


## Declare a C++ library
ADD_HEADER_GROUP(${PROJECT_NAME}_HEADERS)
ADD_SOURCE_GROUP(${PROJECT_NAME}_SOURCES)
ADD_LIBRARY(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_HEADERS})

TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIR} ${OMPL_INCLUDE_DIR})
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} PUBLIC ${OMPL_LIBRARIES})
#INSTALL(TARGETS ${PROJECT_NAME} EXPORT ${TARGETS_EXPORT_NAME} DESTINATION lib)

add_executable(ompl_demo ${SRC_FILES} src/ompl_demo.cpp)
target_link_libraries(ompl_demo ${PROJECT_NAME} ${OMPL_LIBRARIES})
