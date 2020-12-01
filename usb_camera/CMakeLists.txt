cmake_minimum_required(VERSION 3.0.2)
project(usb_camera)


find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        sensor_msgs
        cv_bridge
        )

catkin_package()
