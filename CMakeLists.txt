cmake_minimum_required(VERSION 2.8.3)
project(ur10_armsim)

find_package(catkin REQUIRED
  genmsg
  actionlib_msgs
  geometry_msgs
)

add_action_files(
  FILES
  FlyBall.action
  JointTarget.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
  geometry_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
catkin_package()

install(PROGRAMS
  src/armsim.py
  src/utils.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

