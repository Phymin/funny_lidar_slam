cmake_minimum_required(VERSION 3.10)
project(funny_lidar_slam)

#set(CMAKE_VERBOSE_MAKEFILE true)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

IF (NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF ()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

add_definitions(-DPROJECT_SOURCE_DIR=\"${PROJECT_SOURCE_DIR}\")

# Compiler
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)


set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O3")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
set(ADDITIONAL_CXX_FLAG "-Wall -O3")
set(CMAKE_CXX_FLAGS " ${CMAKE_CXX_FLAGS} ${ADDITIONAL_CXX_FLAG}")

# Eigen
include(cmake/FindEigen.cmake)
include_directories(${EIGEN3_INCLUDE_DIRS})

# Glog
include(cmake/glog.cmake)
include_directories(${GLOG_INCLUDE_DIRS})


# g2o
include(cmake/g2o.cmake)
include_directories(${G2O_INCLUDE_DIRS})

message(STATUS "g2o include: ${G2O_INCLUDE_DIRS} g2o libs: ${G2O_LIBRARIES}, g2o found: ${G2O_FOUND}")

# find_package(
#     catkin REQUIRED COMPONENTS
#     pcl_ros
#     pcl_conversions
#     roscpp
#     sensor_msgs
#     std_msgs
#     message_generation
# )

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
# find_package(G2O REQUIRED)

# livox message
#add_subdirectory(include/3rd/livox_ros_driver)

# add_service_files(FILES save_map.srv)
# generate_messages(DEPENDENCIES std_msgs)
# catkin_package(
#     INCLUDE_DIRS include
#     #  LIBRARIES slam_lib
#     CATKIN_DEPENDS roscpp sensor_msgs std_msgs message_runtime
#     #  DEPENDS system_lib
# )

# include_directories(
#     include
#     ${catkin_INCLUDE_DIRS}
# )

set(SRV_INTERFACES srv_interfaces2)
  rosidl_generate_interfaces(${SRV_INTERFACES}
    "srv/SaveMap.srv"
    DEPENDENCIES builtin_interfaces std_msgs
    LIBRARY_NAME ${PROJECT_NAME}
  )

# rosidl_generate_interfaces(${PROJECT_NAME}
#   "srv/save_map.srv"
#   DEPENDENCIES geometry_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
# )

ament_auto_add_library(
    ${PROJECT_NAME} SHARED
    include/common/keyframe.cpp
    src/optimization/g2o/position_edge.cpp
    src/optimization/g2o/velocity_edge.cpp
    src/optimization/g2o/rotation_edge.cpp
    src/optimization/g2o/gyro_bias_rw_edge.cpp
    src/optimization/g2o/relative_pose_edge.cpp
    src/optimization/g2o/accel_bias_rw_edge.cpp
    src/optimization/g2o/gyro_bias_prior_edge.cpp
    src/optimization/g2o/accel_bias_prior_edge.cpp
    src/optimization/g2o/prior_nav_state_edge.cpp
    src/optimization/g2o/pre_integration_edge.cpp
    src/optimization/g2o/loopclosure_optimizer.cpp
    src/ivox_map/ivox_map.cpp
    src/ivox_map/voxel_grid_node.cpp
    src/imu/pre_integration.cpp
    src/imu/data_synchronizer.cpp
    src/lidar/lidar_model.cpp
    src/lidar/lidar_distortion_corrector.cpp
    src/loam/feature_extractor.cpp
    src/loam/pointcloud_projector.cpp
    src/slam/config_parameters.cpp
    src/slam/preprocessing.cpp
    src/slam/localization.cpp
    src/slam/loop_closure.cpp
    src/slam/split_map.cpp
    src/slam/frontend.cpp
    src/slam/system.cpp
)

rosidl_get_typesupport_target(cpp_typesupport_target
    ${SRV_INTERFACES} "rosidl_typesupport_cpp")

target_link_libraries(
    ${PROJECT_NAME} 
    # tbb
    ${GLOG_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    ${FLANN_LIBRARY}
    ${G2O_LIBRARIES}
    fmt::fmt
    "${cpp_typesupport_target}"
)

ament_auto_add_executable(${PROJECT_NAME}_node app/lidar_slam_app.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

## TEST ###
option(build_unit_tests "will execute all unit tests" ON)

if (build_unit_tests)
    enable_testing()
    add_subdirectory(test)
endif ()

ament_auto_package(INSTALL_TO_SHARE launch config scripts)