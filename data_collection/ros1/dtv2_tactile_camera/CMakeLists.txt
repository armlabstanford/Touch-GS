cmake_minimum_required(VERSION 2.8.3)
project(dtv2_tactile_camera)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_EXTENSIONS OFF)
find_package (PCL 1.10 REQUIRED)

# set(PCL_INCLUDE_DIRS /usr/local/include/pcl-1.12 /usr/include/eigen3/)  # pcl1.12

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})



find_package(catkin REQUIRED COMPONENTS
  roscpp
	rospy
	std_msgs
	cv_bridge 
	dynamic_reconfigure 
	image_transport 
	std_msgs 
	std_srvs 
	sensor_msgs 
	camera_info_manager
	pluginlib
	geometric_shapes
	rosbag
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
)
## pkg-config libraries
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(avcodec libavcodec REQUIRED)
pkg_check_modules(swscale libswscale REQUIRED)

find_package(Boost REQUIRED system filesystem date_time thread)
# find_package(Boost REQUIRED python3)

# Find python and Boost - both are required dependencies
find_package(PythonLibs 3 REQUIRED)
# find_package(Boost COMPONENTS python REQUIRED)


# find_package(OpenCV REQUIRED COMPONENTS core)
message(STATUS "opencv version ${OpenCV_VERSION}")
#if(OpenCV_VERSION VERSION_LESS "4.3.0")
if(OpenCV_VERSION_MAJOR VERSION_EQUAL 4)
  find_package(OpenCV 4 REQUIRED COMPONENTS core imgcodecs videoio)
else()
  find_package(OpenCV 4 REQUIRED COMPONENTS core imgcodecs videoio)
endif()


###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    message_runtime
    tf2_geometry_msgs
  DEPENDS
    EIGEN3
)


###########
## Build ##
###########
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


include_directories(include
${PCL_INCLUDE_DIRS}
${THIS_PACKAGE_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${avcodec_INCLUDE_DIRS}
  ${swscale_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS}
${Boost_INCLUDE_DIR}
${PCL_LIBRARIES}
)

## Build the USB camera library
add_library(${PROJECT_NAME} src/usb_cam.cpp)
target_link_libraries(${PROJECT_NAME}
${THIS_PACKAGE_INCLUDE_DIRS}
  ${avcodec_LIBRARIES}
  ${swscale_LIBRARIES}
  ${catkin_LIBRARIES}
${Boost_INCLUDE_DIR}
${OpenCV_LIBRARIES}
${PCL_LIBRARIES}
)

## Declare a cpp executable
add_executable(${PROJECT_NAME}_node src/usb_cam_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
${THIS_PACKAGE_INCLUDE_DIRS}
  ${avcodec_LIBRARIES}
  ${swscale_LIBRARIES}
  ${catkin_LIBRARIES}
${Boost_LIBRARIES}
${OpenCV_LIBRARIES}
${PCL_LIBRARIES}
)




#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_node ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)


## Copy launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
