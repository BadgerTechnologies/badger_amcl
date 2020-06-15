cmake_minimum_required(VERSION 2.8.3)
project(badger_amcl)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED
        COMPONENTS
            roscpp
            tf2_ros
            tf2_geometry_msgs
            tf2_sensor_msgs
            dynamic_reconfigure
            nav_msgs
            badger_file_lib
            pcl_ros
            octomap_msgs
            angles
        )

find_package(Boost REQUIRED)
find_package(OCTOMAP REQUIRED)
find_package(Eigen3 REQUIRED)

# dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/AMCL.cfg
)

catkin_package(
    CATKIN_DEPENDS
        roscpp
        dynamic_reconfigure
        tf2_ros
        tf2_geometry_msgs
        tf2_sensor_msgs
        badger_file_lib
        pcl_ros
        octomap_msgs
        angles
  INCLUDE_DIRS include
  LIBRARIES amcl_sensors amcl_map amcl_pf amcl_node
  DEPENDS
        OCTOMAP
)

include_directories(include/amcl include/amcl/node include/amcl/map include/amcl/sensors include/amcl/pf)
include_directories(
    include
    ${OCTOMAP_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

add_library(badger_amcl_core
    src/amcl/pf/particle_filter.cpp
    src/amcl/pf/pf_kdtree.cpp
    src/amcl/pf/pdf_gaussian.cpp
    src/amcl/map/map.cpp
    src/amcl/map/occupancy_map.cpp
    src/amcl/map/octomap.cpp
    src/amcl/sensors/odom.cpp
    src/amcl/sensors/planar_scanner.cpp
    src/amcl/sensors/point_cloud_scanner.cpp
    src/amcl/node/node_2d.cpp
    src/amcl/node/node_3d.cpp
    src/amcl/node/node.cpp
)
target_link_libraries(badger_amcl_core
    ${OCTOMAP_LIBRARIES}
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    yaml-cpp
)

add_executable(badger_amcl
	src/main.cpp)
add_dependencies(badger_amcl ${PROJECT_NAME}_gencfg)

target_link_libraries(
    badger_amcl
    badger_amcl_core
)

install( TARGETS
    badger_amcl badger_amcl_core
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/amcl/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY examples/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/examples
)
