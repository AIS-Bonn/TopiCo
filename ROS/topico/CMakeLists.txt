cmake_minimum_required(VERSION 3.0.2)
project(topico)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  std_msgs
  nav_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


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
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

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
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

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
generate_dynamic_reconfigure_options(
  cfg/TopiCo.cfg
)

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
#  INCLUDE_DIRS include
#  LIBRARIES topico
#  CATKIN_DEPENDS dynamic_reconfigure roscpp std_msgs nav_msgs
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
#   src/${PROJECT_NAME}/topico.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
set(SOURCES
src/a_O_P.cpp
src/a_O_V.cpp
src/ab_O_P.cpp
src/abc_O_A.cpp
src/abc_O_AP.cpp
src/abc_O_P.cpp
src/abc_O_V.cpp
src/abc_O_VP.cpp
src/abc_T_A.cpp
src/abc_T_P.cpp
src/abc_T_V.cpp
src/abcd_NO_P.cpp
src/abcd_O_P.cpp
src/abcde_NO_AP.cpp
src/abcde_NO_VP.cpp
src/abcde_O_AP.cpp
src/abcde_O_VP.cpp
src/abcde_T_A.cpp
src/abcde_T_P.cpp
src/abcde_T_V.cpp
src/abcdef_NO_VP.cpp
src/abcdef_O_VP.cpp
src/abcdef_T_P.cpp
src/abcdef_T_V.cpp
src/abcdefg_NO_AP.cpp
src/abcdefg_NO_AVP.cpp
src/abcdefg_NO_VP.cpp
src/abcdefg_NTV_AVP.cpp
src/abcdefg_O_AP.cpp
src/abcdefg_O_AVP.cpp
src/abcdefg_O_VP.cpp
src/abcdefg_TA_AVP.cpp
src/abcdefg_TV_AVP.cpp
src/abcdefg_T_A.cpp
src/abcdefg_T_AP.cpp
src/abcdefg_T_AV.cpp
src/abcdefg_T_P.cpp
src/abcdefg_T_V.cpp
src/abcdefg_T_VP.cpp
src/abcdeg_NO_AP.cpp
src/abcdeg_NO_AVP.cpp
src/abcdeg_NO_VP.cpp
src/abcdeg_NTV_AVP.cpp
src/abcdeg_O_AP.cpp
src/abcdeg_O_AVP.cpp
src/abcdeg_O_VP.cpp
src/abcdeg_TV_AVP.cpp
src/abcdeg_T_A.cpp
src/abcdeg_T_AP.cpp
src/abcdeg_T_AV.cpp
src/abcdeg_T_P.cpp
src/abcdeg_T_V.cpp
src/abcef_O_VP.cpp
src/abcef_T_P.cpp
src/abcef_T_V.cpp
src/abcefg_O_AP.cpp
src/abcefg_O_AVP.cpp
src/abcefg_TA_AVP.cpp
src/abcefg_T_A.cpp
src/abcefg_T_AP.cpp
src/abcefg_T_AV.cpp
src/abcefg_T_P.cpp
src/abcefg_T_V.cpp
src/abceg_O_AP.cpp
src/abceg_O_AVP.cpp
src/abceg_TA_AVP.cpp
src/abceg_T_A.cpp
src/abceg_T_AP.cpp
src/abceg_T_AV.cpp
src/abceg_T_P.cpp
src/abceg_T_V.cpp
src/abs.cpp
src/ac_O_A.cpp
src/ac_O_AP.cpp
src/ac_O_AV.cpp
src/ac_O_P.cpp
src/ac_O_V.cpp
src/ac_T_P.cpp
src/ac_T_V.cpp
src/acd_NO_P.cpp
src/acd_O_P.cpp
src/acde_NO_AP.cpp
src/acde_NO_VP.cpp
src/acde_O_AP.cpp
src/acde_O_VP.cpp
src/acde_T_A.cpp
src/acde_T_P.cpp
src/acde_T_V.cpp
src/acdef_NO_VP.cpp
src/acdef_O_VP.cpp
src/acdef_T_P.cpp
src/acdef_T_V.cpp
src/acdefg_NO_AP.cpp
src/acdefg_NO_AVP.cpp
src/acdefg_NO_VP.cpp
src/acdefg_NTA_AVP.cpp
src/acdefg_NTV_AVP.cpp
src/acdefg_O_AP.cpp
src/acdefg_O_AVP.cpp
src/acdefg_O_VP.cpp
src/acdefg_TV_AVP.cpp
src/acdefg_T_A.cpp
src/acdefg_T_AP.cpp
src/acdefg_T_AV.cpp
src/acdefg_T_P.cpp
src/acdefg_T_V.cpp
src/acdeg_NO_AP.cpp
src/acdeg_NO_AVP.cpp
src/acdeg_O_AP.cpp
src/acdeg_O_AVP.cpp
src/acdeg_TV_AVP.cpp
src/acdeg_T_A.cpp
src/acdeg_T_AP.cpp
src/acdeg_T_AV.cpp
src/acdeg_T_P.cpp
src/acdeg_T_V.cpp
src/acef_O_VP.cpp
src/acef_T_P.cpp
src/acef_T_V.cpp
src/acefg_O_AP.cpp
src/acefg_O_AVP.cpp
src/acefg_TA_AVP.cpp
src/acefg_T_A.cpp
src/acefg_T_AP.cpp
src/acefg_T_AV.cpp
src/acefg_T_P.cpp
src/acefg_T_V.cpp
src/aceg_O_AP.cpp
src/aceg_O_AVP.cpp
src/aceg_T_A.cpp
src/aceg_T_AP.cpp
src/aceg_T_AV.cpp
src/aceg_T_P.cpp
src/aceg_T_V.cpp
src/all.cpp
src/any1.cpp
src/anyNonFinite.cpp
src/check.cpp
src/check_feasibility.cpp
src/check_inputs.cpp
src/circshift.cpp
src/combineVectorElements.cpp
src/compensate_global.cpp
src/construct_setp_struct.cpp
src/correct_feasibility.cpp
src/cut_to_time.cpp
src/diff.cpp
src/eig.cpp
src/eigHermitianStandard.cpp
src/eml_int_forloop_overflow_check.cpp
src/evaluate_to_time.cpp
src/evolve_waypoints.cpp
src/find.cpp
src/i64ddiv.cpp
src/indexShapeCheck.cpp
src/minOrMax.cpp
src/mod.cpp
src/nnz.cpp
src/power.cpp
src/printint.cpp
src/repmat.cpp
src/rollout_t.cpp
src/roots.cpp
src/rotate_jerk.cpp
src/rtGetInf.cpp
src/rtGetNaN.cpp
src/rt_nonfinite.cpp
src/select_cases_O.cpp
src/select_cases_TA.cpp
src/select_cases_TV.cpp
src/select_conditions.cpp
src/select_type.cpp
src/simplify_setp.cpp
src/solve_O.cpp
src/solve_T.cpp
src/sort.cpp
src/sortIdx.cpp
src/sqrt.cpp
src/sub2ind.cpp
src/sum.cpp
src/synchronize_trajectory.cpp
src/topico_wrapper.cpp
src/topico_wrapper_data.cpp
src/topico_wrapper_initialize.cpp
src/topico_wrapper_rtwutil.cpp
src/topico_wrapper_terminate.cpp
src/useConstantDim.cpp
src/vAllOrAny.cpp
src/validateattributes.cpp
src/xdhseqr.cpp
src/xdlanv2.cpp
src/xnrm2.cpp
src/xrot.cpp
src/xzggbal.cpp
src/xzhgeqz.cpp
src/xzlanhs.cpp
src/xzlarf.cpp
src/xzlarfg.cpp
src/xzlartg.cpp)

add_executable(${PROJECT_NAME}_node src/topico_node.cpp ${SOURCES})

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

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
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

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
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_topico.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
