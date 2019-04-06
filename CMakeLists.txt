cmake_minimum_required(VERSION 2.8.3)
project(img_pub)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_filters
  sensor_msgs
  cv_bridge
  rosbag
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES img_pub
   CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

# CXX flags 
set(CMAKE_CXX_FLAGS "-ggdb -g -O3 -w -std=c++11 -fpermissive") # -fPIC -fpermissive

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ./src
)
####### OpenCV #######
find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

###########
## Build ##
###########

set(IMG_PUB_SRCS src/img_pub_seq.cpp 
                 src/img_pub.cpp
                 src/img_pub_realsense.cpp
                 src/img_pub_rgbd.cpp
)

####### img_pub library #######
add_library(img_pub ${IMG_PUB_SRCS})
target_link_libraries(img_pub ${catkin_LIBRARIES} ${OpenCV_LIBS})

############### test pub & sub realsense images ####################
add_executable(test_pub_realsense src/test_img_pub_realsense.cpp)
target_link_libraries(test_pub_realsense img_pub)

add_executable(test_sub_realsense src/sub_img_realsense.cpp)
target_link_libraries(test_sub_realsense ${catkin_LIBRARIES} ${OpenCV_LIBS})

############## test pub & sub rgbd images ################
add_executable(test_pub_rgbd src/test_img_pub_rgbd.cpp)
target_link_libraries(test_pub_rgbd img_pub)

############### test pub & sub sequential images ####################
add_executable(test_pub_seq src/test_img_pub_seq.cpp)
target_link_libraries(test_pub_seq img_pub)

add_executable(test_sub_seq src/sub_img_seq.cpp)
target_link_libraries(test_sub_seq ${catkin_LIBRARIES} ${OpenCV_LIBS})

############## operation on bagfile ###################
add_executable(bag_decompress src/bag_decompress.cpp)
target_link_libraries(bag_decompress ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(bag_decompress_rgb_imu src/bag_decompress_rgb_imu.cpp)
target_link_libraries(bag_decompress_rgb_imu ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(bag_decompress_rgbd_imu src/bag_decompress_rgbd_imu.cpp)
target_link_libraries(bag_decompress_rgbd_imu ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(bag_decompress_imu src/bag_decompress_imu.cpp)
target_link_libraries(bag_decompress_imu ${catkin_LIBRARIES})

add_executable(bag_compress src/bag_compress.cpp)
target_link_libraries(bag_compress img_pub ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(bag_compress_rgbd src/bag_compress_rgbd.cpp)
target_link_libraries(bag_compress_rgbd img_pub ${catkin_LIBRARIES} ${OpenCV_LIBS})


add_executable(sr4k_bdat_to_rgbd src/sr4k_bdat_to_rgbd.cpp src/sr4k_reader.cpp)
target_link_libraries(sr4k_bdat_to_rgbd ${OpenCV_LIBS})
## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/img_pub.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/img_pub_node.cpp)

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
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_img_pub.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
