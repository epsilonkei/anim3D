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

## Compile for task 1
set (TASK1_SOURCES src/floor_rigid_collision.cpp)
add_executable(floor_rigid_collision ${TASK1_SOURCES})
target_link_libraries(floor_rigid_collision ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})
