cmake_minimum_required(VERSION 3.1.3)
project(capturedata)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages


set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_EXTENSIONS OFF)

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
  franka_core_msgs
	camera_info_manager
	pluginlib
	geometric_shapes
	rosbag
    pcl_ros
    pcl_conversions
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

find_package(OpenCV REQUIRED COMPONENTS core)
message(STATUS "opencv version ${OpenCV_VERSION}")
if(OpenCV_VERSION VERSION_LESS "4.0.0")
  find_package(OpenCV 3 REQUIRED COMPONENTS core imgcodecs videoio)
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
    franka_core_msgs
  DEPENDS
    EIGEN3
)

###########
## Build ##
###########

include_directories(include
${THIS_PACKAGE_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${avcodec_INCLUDE_DIRS}
  ${swscale_INCLUDE_DIRS}
 ${EIGEN3_INCLUDE_DIRS}
 ${Boost_INCLUDE_DIR}

 )

## Build the USB camera library
#add_library(${PROJECT_NAME} src/usb_cam.cpp)
#target_link_libraries(${PROJECT_NAME}
#${THIS_PACKAGE_INCLUDE_DIRS}
#  ${avcodec_LIBRARIES}
#  ${swscale_LIBRARIES}
#  ${catkin_LIBRARIES}
#${Boost_INCLUDE_DIR}
#${OpenCV_LIBRARIES}
#)

## Declare a cpp executable
## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()
catkin_install_python(PROGRAMS scripts/capture_data.py
                               scripts/knova_capture.py

                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(${PROJECT_NAME} src/capturedata.cpp)
target_link_libraries(${PROJECT_NAME}
  ${THIS_PACKAGE_INCLUDE_DIRS}
  ${catkin_LIBRARIES}
${Boost_INCLUDE_DIR}
${OpenCV_LIBRARIES}
)

add_executable(${PROJECT_NAME}_allegro src/capturedata_allegro.cpp)

target_link_libraries(${PROJECT_NAME}_allegro
  ${catkin_LIBRARIES}
${Boost_INCLUDE_DIR}
${OpenCV_LIBRARIES}
)

add_executable(${PROJECT_NAME}_mocap src/capturedata_mocap.cpp)

target_link_libraries(${PROJECT_NAME}_mocap
  ${catkin_LIBRARIES}
${Boost_INCLUDE_DIR}
${OpenCV_LIBRARIES}
)

add_executable(${PROJECT_NAME}_imgndepth src/capturedata_imgndepth.cpp)

target_link_libraries(${PROJECT_NAME}_imgndepth
  ${catkin_LIBRARIES}
${Boost_INCLUDE_DIR}
${OpenCV_LIBRARIES}
)

add_executable(${PROJECT_NAME}_flightroom src/capturedata_flightroom.cpp)

target_link_libraries(${PROJECT_NAME}_flightroom
  ${catkin_LIBRARIES}
${Boost_INCLUDE_DIR}
${OpenCV_LIBRARIES}
)



add_executable(${PROJECT_NAME}_key_timeref src/keyboard_timeref.cpp)

target_link_libraries(${PROJECT_NAME}_key_timeref
   ${catkin_LIBRARIES}
${Boost_LIBRARIES}
${OpenCV_LIBRARIES}
)
add_executable(${PROJECT_NAME}_key src/keyboard_input.cpp)

target_link_libraries(${PROJECT_NAME}_key
   ${catkin_LIBRARIES}
${Boost_LIBRARIES}
${OpenCV_LIBRARIES}
)

#add_executable(${PROJECT_NAME}_save src/save_img.cpp)
#target_link_libraries(${PROJECT_NAME}_save
#  ${PROJECT_NAME}
#  ${avcodec_LIBRARIES}
#  ${swscale_LIBRARIES}
#  ${catkin_LIBRARIES}
#${Boost_LIBRARIES}
#${OpenCV_LIBRARIES}
#)

#add_executable(${PROJECT_NAME}_node nodes/sthsthsth.cpp)
#target_link_libraries(${PROJECT_NAME}_node
#  ${PROJECT_NAME}
#${THIS_PACKAGE_INCLUDE_DIRS}
#  ${avcodec_LIBRARIES}
#  ${swscale_LIBRARIES}
#  ${catkin_LIBRARIES}
#${Boost_LIBRARIES}
#${OpenCV_LIBRARIES}
#)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}_allegro ${PROJECT_NAME}_allegro
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}_flightroom ${PROJECT_NAME}_flightroom
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}_mocap ${PROJECT_NAME}_mocap
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)


install(TARGETS ${PROJECT_NAME}_key ${PROJECT_NAME}_key
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}_key_timeref ${PROJECT_NAME}_key_timeref
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
