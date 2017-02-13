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
)
set (CMAKE_PREFIX_PATH /home/shenyu/Qt5.7.0/5.7/gcc_64)
find_package(Qt5Widgets REQUIRED)


catkin_package(
 #   INCLUDE_DIRS include
 #   LIBRARIES robot_platform
#  CATKIN_DEPENDS roscpp
 # DEPENDS
)

include_directories(include
    ${yamlcpp_INCLUDE_DIRS} )

add_executable(main_uart_odom src/server/main_uart_odom.cpp
 src/common/modbus.cpp
 src/common/paras.cpp
 src/server/moc_tcp_server.cpp
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
 )
target_link_libraries(main_global_location
 ${catkin_LIBRARIES}
 )


add_executable(main_map_server src/server/main_map_server.cpp
 src/map/mapreadandwrite.cpp
 src/common/map_process.cpp
 )
target_link_libraries(main_map_server
 ${catkin_LIBRARIES}
 yaml-cpp
 )

add_executable(main_pathplan_server src/server/main_pathplan_server.cpp)
target_link_libraries(main_pathplan_server
    ${catkin_LIBRARIES}
    )
add_definitions(-std=gnu++11)

qt5_use_modules(main_uart_odom
  Core
  Network
 )

qt5_use_modules(main_uart_laser
  Core
  Network
)



