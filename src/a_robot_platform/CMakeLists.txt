cmake_minimum_required(VERSION 2.8.3)
project(a_robot_platform)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
 roscpp
 geometry_msgs
 nav_msgs
 sensor_msgs
 std_msgs
 tf
 rosbag
 message_generation
)
set (CMAKE_PREFIX_PATH ~/Qt5.7.0/5.7/gcc_64)
find_package(Qt5Widgets REQUIRED)


# Find Eigen3 (from http://wiki.ros.org/jade/Migration)
find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()


add_message_files(
  FILES
    SubmapList.msg
    TrajectorySubmapList.msg
    SubmapEntry.msg
)


generate_messages(
  DEPENDENCIES
    geometry_msgs
)

catkin_package()



include_directories(include
    ${yamlcpp_INCLUDE_DIRS} )


if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
    message(STATUS "optional:-std=c++11")
endif(CMAKE_COMPILER_IS_GNUCXX)


add_executable(main_uart_odom src/server/main_uart_odom.cpp
 src/common/modbus.cpp
 src/common/paras.cpp
 src/server/moc_tcpsocketserver.cpp
 src/server/tcpsocketserver.cpp
 src/server/uartdriver.cpp
 src/server/uartodom.cpp
 src/server/uartodompthread.cpp
 src/server/udpsocketserver.cpp
 src/server/moc_udpsocketserver.cpp
)
target_link_libraries(main_uart_odom
 ${catkin_LIBRARIES})

add_executable(main_uart_laser src/server/main_uart_laser.cpp

 src/server/uartdriver.cpp
 src/server/uartlaser.cpp
 src/server/udpsocketserver.cpp
 src/server/moc_udpsocketserver.cpp
 src/common/paras.cpp
 )
target_link_libraries(main_uart_laser
 ${catkin_LIBRARIES})


add_executable(main_global_location src/server/main_global_location.cpp
 src/location/amcl_location.cpp
 src/location/amcl/sensors/amcl_laser.cpp
 src/location/amcl/sensors/amcl_odom.cpp
 src/location/amcl/sensors/amcl_sensor.cpp
 src/location/amcl/pf/eig3.c
 src/location/amcl/pf/pf.c
 src/location/amcl/pf/pf_kdtree.c
 src/location/amcl/pf/pf_pdf.c
 src/location/amcl/pf/pf_vector.c
 src/location/amcl/map/map.c
 src/location/amcl/map/map_cspace.cpp
 src/location/amcl/map/map_range.c
 src/location/amcl/map/map_store.c

 src/common/map_process.cpp
 src/location/scan_match/data_point_container.h
 src/location/scan_match/scan_processor.cpp
 src/map/mapreadandwrite.cpp
 )
target_link_libraries(main_global_location
 ${catkin_LIBRARIES}
  yaml-cpp
 )


add_executable(main_map_server src/server/main_map_server.cpp
 src/map/mapreadandwrite.cpp
 src/common/map_process.cpp
 )
target_link_libraries(main_map_server
 ${catkin_LIBRARIES}
 yaml-cpp
 )

add_executable(main_initial_pose_test src/server/main_initial_pose_test.cpp
 src/common/map_process.cpp
 )
target_link_libraries(main_initial_pose_test
 ${catkin_LIBRARIES})


add_executable(main_path_publish src/server/main_path_publish.cpp
 )
target_link_libraries(main_path_publish
 ${catkin_LIBRARIES})


add_executable(main_pathplan_server src/server/main_pathplan_server.cpp)
target_link_libraries(main_pathplan_server
    ${catkin_LIBRARIES}
)


qt5_use_modules(main_uart_odom
  Core
  Network
 )

qt5_use_modules(main_uart_laser
  Core
  Network
)


