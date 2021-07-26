cmake_minimum_required(VERSION 2.8.3)

SET(PROJECT_NAME chomp)
SET(PROJECT_DESCRIPTION "chomp with time opt ")
SET(PROJECT_URL "http://github.com/ggory15/chomp")

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

#COMPUTE_PROJECT_ARGS(PROJECT_ARGS LANGUAGES CXX)
#PROJECT(${PROJECT_NAME} ${PROJECT_ARGS})

find_package(catkin REQUIRED COMPONENTS)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS mzcommon
 DEPENDS system_lib
)

include(FindPkgConfig)
pkg_search_module(EIGEN3 REQUIRED eigen3>=3)
find_package(mzcommon REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIRS} ${mzcommon_INCLUDE_DIRS})

set(CMAKE_C_FLAGS "-Wall -g")
set(CMAKE_CXX_FLAGS "-Wall -g")

set(CMAKE_C_FLAGS_DEBUG "-O")
set(CMAKE_CXX_FLAGS_DEBUG "-O")

set(CMAKE_C_FLAGS_RELEASE "-O2")
set(CMAKE_CXX_FLAGS_RELEASE "-O2")



set(${PROJECT_NAME}_SOURCES
  src/Constraint.cpp
  src/ConstraintFactory.cpp
  src/Chomp.cpp 
  src/Map2D.cpp
)
set(${PROJECT_NAME}_HEADERS
  include/chomp/Chomp.h
  include/chomp/chomputil.h
  include/chomp/Constraint.h
  include/chomp/ConstraintFactory.h
  include/chomp/Map2D.h
)

ADD_SOURCE_GROUP(${PROJECT_NAME}_SOURCES)
ADD_HEADER_GROUP(${PROJECT_NAME}_HEADERS)
ADD_LIBRARY(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_HEADERS})

TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIR} ${mzcommon_INCLUDE_DIRS})
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} PUBLIC)
#INSTALL(TARGETS ${PROJECT_NAME} EXPORT ${TARGETS_EXPORT_NAME} DESTINATION lib)



