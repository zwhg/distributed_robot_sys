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
)

set (CMAKE_PREFIX_PATH /home/shenyu/Qt5.7.0/5.7/gcc_64)
find_package(Qt5Widgets REQUIRED)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_platform
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)



include_directories(
  ${catkin_INCLUDE_DIRS}
)

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

add_executable(main_sub_pose src/server/main_sub_pose.cpp
 src/common/modbus.cpp
 src/common/paras.cpp
 )
target_link_libraries(main_sub_pose
 ${catkin_LIBRARIES})





add_definitions(-std=gnu++11)

qt5_use_modules(main_uart_odom
  Core
  Network
 )

qt5_use_modules(main_uart_laser
  Core
  Network
)

qt5_use_modules(main_sub_pose
  Core
)
