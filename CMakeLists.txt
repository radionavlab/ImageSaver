cmake_minimum_required(VERSION 2.8.3)
project(image_saver)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# Library and headers
set(CAMERA_CLIENT_INCLUDE_DIR "/usr/local/include/quadcam")
set(CAMERA_CLIENT_LIBS "/usr/local/lib/quadcam/libcamera_client.a")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED 
    COMPONENTS
    roscpp
    mg_msgs
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
#  LIBRARIES image_saver
    CATKIN_DEPENDS mg_msgs
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    src
    ${CAMERA_CLIENT_INCLUDE_DIR}
# ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/main.cc src/node.cc src/gps.cc src/image_saver.cc)


## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${CAMERA_CLIENT_LIBS}
)

add_dependencies(${PROJECT_NAME} mg_msgs_gencpp)
