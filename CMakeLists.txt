cmake_minimum_required(VERSION 2.8.3)
project(laserscan_merger)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS laser_geometry roscpp sensor_msgs std_msgs tf2 tf2_ros dynamic_reconfigure pcl_ros)

find_package(Eigen3 REQUIRED)

find_package(PCL REQUIRED)

# bugfix for g++-Warning
#   "<command-line>:0:0: warning: missing whitespace after the macro name"
remove_definitions(-DDISABLE_LIBUSB-1.0)

generate_dynamic_reconfigure_options(cfg/laserscan_merger.cfg)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS
#  LIBRARIES laser_merger
#  CATKIN_DEPENDS laser_geometry roscpp sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)



add_executable(laserscan_merger src/laserscan_merger.cpp)
target_link_libraries(laserscan_merger ${catkin_LIBRARIES} ${PCL_LIBRARIES})

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(laserscan_merger ${PROJECT_NAME}_gencfg)


add_executable(republish_scans src/republish_scans.cpp)
target_link_libraries(republish_scans ${catkin_LIBRARIES})


#############
## Install ##
#############


## Mark executables and/or libraries for installation
install(TARGETS laserscan_merger republish_scans
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)


