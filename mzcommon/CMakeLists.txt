cmake_minimum_required(VERSION 2.8.3)

SET(PROJECT_NAME mzcommon)
SET(PROJECT_DESCRIPTION "kimm_trajectry with time opt ")
SET(PROJECT_URL "http://github.com/ggory15/kim_trajectory")

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
#INCLUDE(cmake/mzsettings.cmake)

#COMPUTE_PROJECT_ARGS(PROJECT_ARGS LANGUAGES CXX)
#PROJECT(${PROJECT_NAME} ${PROJECT_ARGS})

find_package(catkin REQUIRED COMPONENTS)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 DEPENDS system_lib
)

find_library(OPENGL_LIBRARY GL)
find_library(GLU_LIBRARY GLU)
set(OPENGL_LIBRARY ${OPENGL_LIBRARY} ${GLU_LIBRARY})
  
include(FindPkgConfig)

find_library(PNG_LIBRARY png)
find_library(EXPAT_LIBRARY expat)
find_library(GLUT_LIBRARY glut)

pkg_search_module(EIGEN3 REQUIRED eigen3>=3)
pkg_search_module(CCD libccd ccd)
pkg_search_module(CAIRO cairo)

if (CAIRO_FOUND)
	add_definitions(-DMZ_HAVE_CAIRO)
endif ()

include_directories(include ${EIGEN3_INCLUDE_DIRS})

set(CMAKE_C_FLAGS "-Wall -g")
set(CMAKE_CXX_FLAGS "-Wall -g")

set(CMAKE_C_FLAGS_DEBUG "-O")
set(CMAKE_CXX_FLAGS_DEBUG "-O")

set(CMAKE_C_FLAGS_RELEASE "-O2")
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

set(${PROJECT_NAME}_SOURCES
  src/mersenne.c
  src/gauss.c
  src/GlCamera.cpp
  src/GlFrameGrabber.cpp
  src/GlTexture2D.cpp
  src/MzGlutApp.cpp
  src/TinyDom.cpp
  src/TriMesh3.cpp
  src/HeightMap.cpp
  src/DtGrid.cpp
  src/Geom2.cpp
  src/glstuff.cpp
  src/CCDWrapper.cpp)

if (GLEW_FOUND)
  set(mzcommon_srcs ${mzcommon_srcs} GlFramebufferObject.cpp)
endif()

set(BASE_LIBS
  ${OPENGL_LIBRARY}
  ${CCD_LDFLAGS}
)

set(GUI_LIBS
  ${BASE_LIBS}
  ${GLUT_LIBRARY}
  ${PNG_LIBRARY}
)
set(${PROJECT_NAME}_HEADERS
	${CMAKE_INCLUDE_DIR}
	${PROJECT_INCLUDE_DIR}
	include/mzcommon/Affine2.h
  include/mzcommon/AngleUtil.h
  include/mzcommon/assert_equal.h
  include/mzcommon/bittwiddle.h
  include/mzcommon/Box2.h
  include/mzcommon/Box3.h
  include/mzcommon/Bresenham.h
  include/mzcommon/CCDWrapper.h
  include/mzcommon/DtGrid.h
  include/mzcommon/gauss.h
  include/mzcommon/Geom2.h
  include/mzcommon/GlCamera.h
  include/mzcommon/GlFramebufferObject.h
  include/mzcommon/GlFrameGrabber.h
  include/mzcommon/glstuff.h
  include/mzcommon/GlTexture2D.h
  include/mzcommon/Grid.h
  include/mzcommon/Gradient.h
  include/mzcommon/Grid2.h
  include/mzcommon/Grid3.h
  include/mzcommon/HeightMap.h
  include/mzcommon/mat2.h
  include/mzcommon/mat3.h
  include/mzcommon/mat4.h
  include/mzcommon/mersenne.h
  include/mzcommon/MzGlutApp.h
  include/mzcommon/quat.h
  include/mzcommon/QuickPNG.h
  include/mzcommon/SimpleConfig.h
  include/mzcommon/SimpleTokenizer.h
  include/mzcommon/strutils.h
  include/mzcommon/TimeUtil.h
  include/mzcommon/TinyDom.h

  include/mzcommon/Transform2.h
  include/mzcommon/Transform3.h
  include/mzcommon/TriMesh3.h
  include/mzcommon/vec2.h
  include/mzcommon/vec2u.h
  include/mzcommon/vec3.h
  include/mzcommon/vec3u.h
  include/mzcommon/vec4.h



	)

ADD_HEADER_GROUP(${PROJECT_NAME}_HEADERS)
ADD_SOURCE_GROUP(${PROJECT_NAME}_SOURCES)
ADD_LIBRARY(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_HEADERS})

TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIR} ${OMPL_INCLUDE_DIR})
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} PUBLIC ${OPENGL_LIBRARY} ${GLUT_LIBRARY} ${EXPAT_LIBRARY} ${PNG_LIBRARY} ${CCD_LDFLAGS})
#INSTALL(TARGETS ${PROJECT_NAME} EXPORT ${TARGETS_EXPORT_NAME} DESTINATION lib)



