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
set (TASK1_SOURCES src/particle_under_grav.cpp)
add_executable(particle_under_grav ${TASK1_SOURCES})
target_link_libraries(particle_under_grav ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})

## Compile for task 2
set (TASK2_SOURCES src/floor_collision.cpp)
add_executable(floor_collision ${TASK2_SOURCES})
target_link_libraries(floor_collision ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})

## Compile for task 3
set (TASK3_SOURCES src/line_segment.cpp)
add_executable(line_segment ${TASK3_SOURCES})
target_link_libraries(line_segment ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})

## Compile for task 4
set (TASK4_SOURCES src/billiard_sim.cpp)
add_executable(billiard_sim ${TASK4_SOURCES})
target_link_libraries(billiard_sim ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})

## Compile for task 5
set (TASK5_SOURCES src/gravitational_field.cpp)
add_executable(gravitational_field ${TASK5_SOURCES})
target_link_libraries(gravitational_field ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})

## Compile for hacker point 1
set (BHSIM_SOURCES src/gravitational_field_fast.cpp)
add_executable(gravitational_field_fast ${BHSIM_SOURCES})
target_link_libraries(gravitational_field_fast ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})

## Compile for hacker point 2
set (NEWTON_CRADLE_SOURCES src/newton_cradle.cpp)
add_executable(newton_cradle ${NEWTON_CRADLE_SOURCES})
target_link_libraries(newton_cradle ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})
