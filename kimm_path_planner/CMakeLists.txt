cmake_minimum_required(VERSION 3.0.2)

SET(PROJECT_NAME kimm_path_planner)
SET(PROJECT_DESCRIPTION "kimm_path_planning")
SET(PROJECT_URL "http://github.com/ggory15/kimm_path_planning")

# Disable -Werror on Unix for now.
SET(CXX_DISABLE_WERROR TRUE)
SET(CMAKE_VERBOSE_MAKEFILE TRUE)
SET(PROJECT_USE_CMAKE_EXPORT TRUE)
add_compile_options(-std=c++17 -Ofast)

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
 CATKIN_DEPENDS mzcommon chomp steering_functions
 DEPENDS system_lib
)

include(FindPkgConfig)

remove_definitions(-Wignored-attributes)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# --- OPTIONS ----------------------------------------
OPTION (BUILD_UNIT_TESTS "Build the unitary tests" ON)
OPTION (INITIALIZE_WITH_NAN "Initialize Eigen entries with NaN" OFF)
OPTION (EIGEN_RUNTIME_NO_MALLOC "If ON, it can assert in case of runtime allocation" ON)
OPTION (EIGEN_NO_AUTOMATIC_RESIZING "If ON, it forbids automatic resizing of dynamics arrays and matrices" OFF)

IF(INITIALIZE_WITH_NAN)
  MESSAGE(STATUS "Initialize with NaN all the Eigen entries.")
  ADD_DEFINITIONS(-DEIGEN_INITIALIZE_MATRICES_BY_NAN)
ENDIF(INITIALIZE_WITH_NAN)

IF(EIGEN_RUNTIME_NO_MALLOC)
  MESSAGE(STATUS "Option EIGEN_RUNTIME_NO_MALLOC on.")
  ADD_DEFINITIONS(-DEIGEN_RUNTIME_NO_MALLOC)
ENDIF(EIGEN_RUNTIME_NO_MALLOC)

IF(EIGEN_NO_AUTOMATIC_RESIZING)
  MESSAGE(STATUS "Option EIGEN_NO_AUTOMATIC_RESIZING on.")
  ADD_DEFINITIONS(-DEIGEN_NO_AUTOMATIC_RESIZING)
ENDIF(EIGEN_NO_AUTOMATIC_RESIZING)


find_package(Eigen3 REQUIRED NO_MODULE)
include_directories(include ${EIGEN3_INCLUDE_DIR})
find_package(Boost REQUIRED COMPONENTS system)
find_package(ompl 1.5 REQUIRED)
include_directories(include ${OMPL_INCLUDE_DIRS})

find_package(steering_functions REQUIRED)
include_directories(${steering_functions_INCLUDE_DIRS})

find_package(mzcommon REQUIRED)
include_directories(${mzcommon_INCLUDE_DIRS})

find_package(chomp REQUIRED)
include_directories(${chomp_INCLUDE_DIRS})

find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

SET(${PROJECT_NAME}_BASE_HEADERS
  include/kimm_path_planner/base/Environment.h
  include/kimm_path_planner/base/Primitives.h
  include/kimm_path_planner/base/GridMaze.h
  include/kimm_path_planner/base/PlannerSettings.h
  include/kimm_path_planner/base/PathStatistics.hpp

  include/kimm_path_planner/base/PathStatistics.hpp
  include/kimm_path_planner/base/PolygonMaze.h
  include/kimm_path_planner/base/PlannerConfigurator.hpp
  include/kimm_path_planner/base/EnvironmentStateValidityChecker.h
  include/kimm_path_planner/base/TimedResult.hpp
  )

SET(${PROJECT_NAME}_UTIL_HEADERS
  include/kimm_path_planner/utils/PathEvaluation.hpp
  include/kimm_path_planner/utils/ScenarioLoader.h
  include/kimm_path_planner/utils/Stopwatch.hpp

  include/kimm_path_planner/utils/PlannerUtils.hpp
  include/kimm_path_planner/utils/SvgPolygonLoader.hpp
  include/kimm_path_planner/utils/OptimizationObjective.h
  include/kimm_path_planner/utils/Log.h
)

SET(${PROJECT_NAME}_THIRD_HEADERS
  include/kimm_path_planner/third_party/params/params.hpp
  include/kimm_path_planner/third_party/collision2d/sat.hpp
)

SET(${PROJECT_NAME}_MODEL_HEADERS
  include/kimm_path_planner/fp_models/ForwardPropagation.h
  include/kimm_path_planner/fp_models/kinematic_car/kinematic_car.hpp
  include/kimm_path_planner/fp_models/kinematic_single_track/kinematic_single_track.hpp
)

SET(${PROJECT_NAME}_STEER_HEADERS
  include/kimm_path_planner/steer_functions/Steering.h
  include/kimm_path_planner/steer_functions/POSQ/POSQ.hpp
  include/kimm_path_planner/steer_functions/POSQ/POSQStateSpace.h
)

SET(${PROJECT_NAME}_SMOOTHER_HEADERS
  include/kimm_path_planner/smoothers/chomp/CHOMP.h
  include/kimm_path_planner/smoothers/grips/GRIPS.h
  include/kimm_path_planner/smoothers/ompl/OmplSmoother.hpp
)

SET(${PROJECT_NAME}_PLANNERS_HEADERS
  include/kimm_path_planner/planners/AbstractPlanner.h
  include/kimm_path_planner/planners/OMPLControlPlanner.hpp
  include/kimm_path_planner/planners/OMPLPlanner.hpp

  include/kimm_path_planner/planners/thetastar/gnode_base.h
  include/kimm_path_planner/planners/thetastar/gnode.h
  include/kimm_path_planner/planners/thetastar/stl_thetastar.h
  include/kimm_path_planner/planners/thetastar/ThetaStar.h


)

SET(${PROJECT_NAME}_METRIC_HEADERS
  include/kimm_path_planner/metrics/ClearingMetric.h
  include/kimm_path_planner/metrics/TrajectoryMetric.h
  include/kimm_path_planner/metrics/MaxCurvatureMetric.h
  include/kimm_path_planner/metrics/PathLengthMetric.h
  include/kimm_path_planner/metrics/AOLMetric.h
  include/kimm_path_planner/metrics/NormalizedCurvatureMetric.h
)



SET(${PROJECT_NAME}_HEADERS
  ${${PROJECT_NAME}_BASE_HEADERS}
  ${${PROJECT_NAME}_UTIL_HEADERS}
  ${${PROJECT_NAME}_THIRD_HEADERS}
  ${${PROJECT_NAME}_MODEL_HEADERS}
  ${${PROJECT_NAME}_STEER_HEADERS}
  ${${PROJECT_NAME}_METRIC_HEADERS}
  ${${PROJECT_NAME}_SMOOTHER_HEADERS}
  ${${PROJECT_NAME}_PLANNERS_HEADERS}
  
)


SET(${PROJECT_NAME}_SOURCES
  src/utils/OptimizationObjective.cpp
  src/utils/ScenarioLoader.cpp
  src/utils/Log.cpp

  src/steer_functions/POSQ/POSQStateSpace.cpp

  src/base/Environment.cpp
  src/base/EnvironmentStateValidityChecker.cpp
  src/base/Primitives.cpp
  src/base/PlannerSettings.cpp

  src/planners/AbstractPlanner.cpp
  src/planners/thetastar/gnode_base.cpp
  src/planners/thetastar/ThetaStar.cpp

  src/base/GridMaze.cpp
  src/base/PolygonMaze.cpp

  src/smoothers/grips/GRIPS.cpp
  src/smoothers/chomp/CHOMP.cpp
  )

ADD_SOURCE_GROUP(${PROJECT_NAME}_HEADERS)
ADD_SOURCE_GROUP(${PROJECT_NAME}_SOURCES)
ADD_LIBRARY(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_HEADERS})

set(EXTRA_LIB ${EXTRA_LIB} ${SBPL_LIBRARIES} ${Boost_LIBRARIES} ${OMPL_LIBRARIES} ${chomp_LIBRARIES} ${mzcommon_LIBRARIES} ${steering_functions_LIBRARIES})
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIR} ${OMPL_INCLUDE_DIRS} ${chomp_INCLUDE_DIR} ${steering_functions_INCLUDE_DIRS})
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} PUBLIC ${EXTRA_LIB})
add_dependencies(${PROJECT_NAME}  ${${PROJECT_NAME}_EXPORTEHQPD_TARGETS} ${catkin_EXPORTED_TARGETS}  )
#INSTALL(TARGETS ${PROJECT_NAME} EXPORT ${TARGETS_EXPORT_NAME} DESTINATION lib)


add_executable(utest unittest/environment.cpp)
target_link_libraries(utest ${PROJECT_NAME} ${GTEST_LIBRARIES} ${catkin_LIBRARIES} ${EXTRA_LIB} pthread)

#INSTALL(FILES package.xml DESTINATION share/${PROJECT_NAME})

