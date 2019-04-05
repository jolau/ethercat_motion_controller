cmake_minimum_required(VERSION 2.8.3)
project(varileg_lowlevel_controller)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  COMPONENTS
  any_node
  message_logger
  soem_interface
  varileg_msgs
  )

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES
  ${PROJECT_NAME}
  CATKIN_DEPENDS
  any_node
  std_msgs
  message_logger
  soem_interface
  varileg_msgs
)

###########
## Build ##
###########
add_definitions(-std=c++11)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/EthercatNode.cpp
  src/EposEthercatSlave.cpp
  include/varileg_lowlevel_controller/RxPdo.hpp
  include/varileg_lowlevel_controller/TxPdo.hpp
  src/VarilegEthercatBusManager.cpp
  include/varileg_lowlevel_controller/EposCommandLibrary.hpp
  src/EposEthercatSlaveManager.cpp
  include/varileg_lowlevel_controller/EposEthercatSlaveManager.hpp
  src/examples/EposExampleNode.cpp
  include/varileg_lowlevel_controller/ExtendedJointState.hpp
  include/varileg_lowlevel_controller/DeviceState.hpp
  include/varileg_lowlevel_controller/OperatingMode.hpp
  include/varileg_lowlevel_controller/HomingState.hpp include/varileg_lowlevel_controller/HomingMethod.hpp)


target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  )

add_executable(${PROJECT_NAME}_node
  src/varileg_lowlevel_controller.cpp
  )

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  )

add_executable(epos_example_node
  src/examples/epos_example.cpp
  )

target_link_libraries(epos_example_node
  ${PROJECT_NAME}
  )

#add_dependencies(
#  ${PROJECT_NAME}_example_node
#  ${PROJECT_NAME}
#  ${catkin_EXPORTED_TARGETS}
#)

#############
## Install ##
#############
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
  )
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY
  launch
  param
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )