cmake_minimum_required(VERSION 2.8.3)
project(a05_ultrasonic)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  serial
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs serial sensor_msgs
  INCLUDE_DIRS include
  LIBRARIES a05_ultrasonic
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(a05_ultrasonic
  src/a05_ultrasonic.cpp
)

add_executable(a05_ultrasonic_node
  src/a05_ultrasonic_node.cpp
)

target_link_libraries(a05_ultrasonic_node
  ${catkin_LIBRARIES}
  a05_ultrasonic
)

# Add explicit linking to serial library
target_link_libraries(a05_ultrasonic
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

install(TARGETS a05_ultrasonic a05_ultrasonic_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)