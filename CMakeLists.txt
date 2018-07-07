cmake_minimum_required(VERSION 2.8)
project(Animation3D)

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
include_directories( ${OPENGL_INCLUDE_DIRS}  ${GLUT_INCLUDE_DIRS} )

# find_package(Eigen REQUIRED)
# include_directories(${Eigen_INCLUDE_DIRS})
# include_directories(${EIGEN_INCLUDE_DIRS})
find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIR})
include_directories(/usr/include/eigen3)

## Compile for demo 1
set (DEMO1_SOURCES src/water_demo.cpp)
add_executable(water_demo ${DEMO1_SOURCES})
target_link_libraries(water_demo ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})
