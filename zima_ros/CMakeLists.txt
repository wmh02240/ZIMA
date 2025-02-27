cmake_minimum_required(VERSION 3.0.2)
project(zima_ros)

set(CMAKE_BUILD_TYPE "Release")
message(STATUS "Build for ${CMAKE_BUILD_TYPE}")

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-Wall)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  tf
  # For support cartographer.
  # cartographer_ros_msgs
  # For support zima gazebo robot
  gazebo_msgs
  rosgraph_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Protobuf REQUIRED)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  ModeSwitch.msg
  VirtualWall.msg
  BlockArea.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
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
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES zima_ros
 CATKIN_DEPENDS
  roscpp
  rospy
  std_msgs
#  cartographer_ros_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/zima_ros.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure

set(zima_DYNAMIC_LIBRARIES "-lzima_base -lzima_core ${PROTOBUF_LIBRARY} -lgflags -lglog -lpthread")
set(zima_STATIC_LIBRARIES "-lzima_core_static -lzima_base_static ${PROTOBUF_LIBRARY} -lgflags -lglog -lpthread")
set(zima_LIBRARIES ${zima_STATIC_LIBRARIES})
# add_executable(zima_path_planner_test
#   src/zima_path_planner_test.cpp
#   src/gazebo/zima_gazebo_node.cpp
#   src/util.cpp
#   src/zima_node.cpp)
# add_dependencies(zima_path_planner_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(zima_path_planner_test ${catkin_LIBRARIES} ${zima_LIBRARIES})

# add_executable(zima_section_cleaning_simulation_test
#   src/zima_section_cleaning_simulation_test.cpp
#   src/gazebo/zima_gazebo_node.cpp
#   src/util.cpp
#   src/zima_node.cpp)
# add_dependencies(zima_section_cleaning_simulation_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(zima_section_cleaning_simulation_test ${catkin_LIBRARIES} ${zima_LIBRARIES})

# add_executable(zima_room_cleaning_simulation_test
#   src/zima_room_cleaning_simulation_test.cpp
#   src/gazebo/zima_gazebo_node.cpp
#   src/util.cpp
#   src/zima_node.cpp)
# add_dependencies(zima_room_cleaning_simulation_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(zima_room_cleaning_simulation_test ${catkin_LIBRARIES} ${zima_LIBRARIES})

# add_executable(zima_house_cleaning_with_user_block_simulation_test
#   src/zima_house_cleaning_with_user_block_simulation_test.cpp
#   src/gazebo/zima_gazebo_node.cpp
#   src/util.cpp
#   src/zima_node.cpp)
# add_dependencies(zima_house_cleaning_with_user_block_simulation_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(zima_house_cleaning_with_user_block_simulation_test ${catkin_LIBRARIES} ${zima_LIBRARIES})

# add_executable(
#   zima_house_cleaning_simulation_test
#   src/zima_house_cleaning_simulation_test.cpp
#   src/gazebo/zima_gazebo_node.cpp
#   src/util.cpp
#   src/zima_node.cpp)
# add_dependencies(zima_house_cleaning_simulation_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(zima_house_cleaning_simulation_test ${catkin_LIBRARIES} ${zima_LIBRARIES})

# add_executable(
#   zima_user_select_area_cleaning_simulation_test
#   src/zima_user_select_area_cleaning_simulation_test.cpp
#   src/gazebo/zima_gazebo_node.cpp
#   src/util.cpp
#   src/zima_node.cpp)
# add_dependencies(zima_user_select_area_cleaning_simulation_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(zima_user_select_area_cleaning_simulation_test ${catkin_LIBRARIES} ${zima_LIBRARIES})

# add_executable(
#   zima_user_select_room_cleaning_simulation_test
#   src/zima_user_select_room_cleaning_simulation_test.cpp
#   src/gazebo/zima_gazebo_node.cpp
#   src/util.cpp
#   src/zima_node.cpp)
# add_dependencies(zima_user_select_room_cleaning_simulation_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(zima_user_select_room_cleaning_simulation_test ${catkin_LIBRARIES} ${zima_LIBRARIES})

add_executable(
  zima_mode_gazebo_test
  src/gazebo/zima_mode_gazebo_test.cpp
  src/gazebo/zima_gazebo_node.cpp
  src/util.cpp
  src/zima_ros_version.cpp
  src/zima_node.cpp)
add_dependencies(zima_mode_gazebo_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(zima_mode_gazebo_test ${catkin_LIBRARIES} ${zima_LIBRARIES})

# add_executable(
#   zima_matcher_gazebo_test
#   src/gazebo/zima_matcher_gazebo_test.cpp
#   src/gazebo/zima_gazebo_node.cpp
#   src/util.cpp
#   src/zima_node.cpp)
# add_dependencies(zima_matcher_gazebo_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(zima_matcher_gazebo_test ${catkin_LIBRARIES} ${zima_LIBRARIES})

# add_executable(
#   zima_xtark_test
#   src/xtark/zima_xtark_test.cpp
#   src/xtark/zima_xtark_node.cpp
#   src/util.cpp
#   src/zima_node.cpp)
# add_dependencies(zima_xtark_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(zima_xtark_test ${catkin_LIBRARIES} ${zima_LIBRARIES})

# add_executable(
#   zima_mode_kobuki_test
#   src/kobuki/zima_kobuki_node.cpp
#   src/kobuki/zima_mode_kobuki_test.cpp
#   src/util.cpp
#   src/zima_node.cpp)
# add_dependencies(zima_mode_kobuki_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(zima_mode_kobuki_test ${catkin_LIBRARIES} ${zima_LIBRARIES})

# add_executable(
#   zima_matcher_kobuki_test
#   src/kobuki/zima_kobuki_node.cpp
#   src/kobuki/zima_matcher_kobuki_test.cpp
#   src/util.cpp
#   src/zima_node.cpp)
# add_dependencies(zima_matcher_kobuki_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(zima_matcher_kobuki_test ${catkin_LIBRARIES} ${zima_LIBRARIES})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/zima_ros_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS
  zima_mode_gazebo_test
  # zima_mode_kobuki_test
  # zima_matcher_kobuki_test
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(DIRECTORY
#   launch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
install(FILES
  launch/gazebo_demo.launch
  launch/rviz.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(FILES
  zima.rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_zima_ros.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
