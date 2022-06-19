cmake_minimum_required(VERSION 2.8.3)
project(radar_ego_velocity_estimator)

find_package(catkin_simple REQUIRED)
catkin_simple()

add_compile_options(-std=c++14)

SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -O0 -g")
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -march=native")

enable_language(Fortran CXX)

add_library(odrpack
    src/odrpack/d_odr.f
    src/odrpack/d_mprec.f
    src/odrpack/lpkbls.f
    src/odrpack/dlunoc.f
    src/odrpack/real_precision.f
)

cs_add_library(${PROJECT_NAME}
   src/simple_profiler.cpp
   src/radar_point_cloud.cpp
   src/odr.cpp
   src/radar_ego_velocity_estimator.cpp
   src/radar_body_velocity_estimator.cpp
   src/radar_ego_velocity_estimator_ros.cpp
   src/radar_body_velocity_estimator_ros.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  odrpack
  )

cs_add_executable(radar_ego_velocity_estimation_ros_node src/nodes/radar_ego_velocity_estimation_ros_node.cpp )
target_link_libraries(radar_ego_velocity_estimation_ros_node ${PROJECT_NAME})

cs_add_executable(radar_ego_velocity_estimation_rosbag_node src/nodes/radar_ego_velocity_estimation_rosbag_node.cpp )
target_link_libraries(radar_ego_velocity_estimation_rosbag_node ${PROJECT_NAME})

cs_add_executable(radar_body_velocity_estimation_ros_node src/nodes/radar_body_velocity_estimation_ros_node.cpp )
target_link_libraries(radar_body_velocity_estimation_ros_node ${PROJECT_NAME})

cs_add_executable(radar_body_velocity_estimation_rosbag_node src/nodes/radar_body_velocity_estimation_rosbag_node.cpp )
target_link_libraries(radar_body_velocity_estimation_rosbag_node ${PROJECT_NAME})

cs_install()

cs_export(INCLUDE_DIRS include)

## qtcreator project tree
FILE(GLOB_RECURSE LibFiles "include/*")
add_custom_target(headers SOURCES ${LibFiles})

file(GLOB_RECURSE CFG "cfg/*")
add_custom_target(CFG SOURCES ${CFG})

file(GLOB_RECURSE LAUNCH "launch/*")
add_custom_target(LAUNCH SOURCES ${LAUNCH})

file(GLOB_RECURSE PYTHON "python/*")
add_custom_target(PYTHON SOURCES ${PYTHON})

file(GLOB_RECURSE CONFIG "config/*")
add_custom_target(CONFIG SOURCES ${CONFIG})
