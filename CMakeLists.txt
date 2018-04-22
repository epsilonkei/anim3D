cmake_minimum_required(VERSION 2.8)
project(Animation3D)

set (ANIM_SOURCES src/anim3D.cpp)
add_executable(anim3D ${ANIM_SOURCES})
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
include_directories( ${OPENGL_INCLUDE_DIRS}  ${GLUT_INCLUDE_DIRS} )

# find_package(Eigen REQUIRED)
# find_package(Boost REQUIRED)
# include_directories(${Eigen_INCLUDE_DIRS})
# include_directories(${EIGEN_INCLUDE_DIRS})
# include_directories(${Boost_INCLUDE_DIR})
include_directories(/usr/include/eigen3)

target_link_libraries(anim3D ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})
