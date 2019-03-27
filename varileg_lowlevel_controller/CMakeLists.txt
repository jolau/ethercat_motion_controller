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
  include/varileg_lowlevel_controller/RxPdo.hpp include/varileg_lowlevel_controller/TxPdo.hpp src/VarilegEthercatBusManager.cpp include/varileg_lowlevel_controller/VarilegEthercatBusManager.hpp include/varileg_lowlevel_controller/EposCommandLibrary.hpp)


target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node
  src/varileg_lowlevel_controller.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  )

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