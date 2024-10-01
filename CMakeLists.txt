cmake_minimum_required(VERSION 3.0.2)
project(ekf_hsr)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()


###################################
## catkin specific configuration ##
###################################
## Declare things to be passed to dependent projects
## CATKIN_DEPENDS: catkin packages dependent projects also need
catkin_package(
  CATKIN_DEPENDS rospy sensor_msgs std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
catkin_install_python(PROGRAMS
  src/publisher1.py
  src/minscan.py
  src/minscanrviz.py
  src/minscanparams.py
  src/minscanparamsmove.py
  src/module.py
  src/modA.py
  src/tf_listener.py
  src/hsr_to_neutral.py
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)
