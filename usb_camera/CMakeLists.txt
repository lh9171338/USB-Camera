cmake_minimum_required(VERSION 2.8.3)
project(usb_camera)

find_package(OpenCV 3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        sensor_msgs
        image_transport
        cv_bridge
        )


catkin_package()

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)


add_executable(video_capture src/video_capture.cpp)
add_executable(video_view src/video_view.cpp)
target_link_libraries(video_capture ${OpenCV_LIBS} ${catkin_LIBRARIES})
target_link_libraries(video_view ${OpenCV_LIBS} ${catkin_LIBRARIES})
