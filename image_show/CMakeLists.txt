cmake_minimum_required(VERSION 2.8.3)
project(image_show)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(OpenCV 3 REQUIRED)
message(STATUS "OpenCV version: ${OpenCV_VERSION}")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        sensor_msgs
        image_transport
        cv_bridge
        )


catkin_package(
        #  INCLUDE_DIRS include
        CATKIN_DEPENDS sensor_msgs
        #  DEPENDS system_lib
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${catkin_LIBRARIES})
