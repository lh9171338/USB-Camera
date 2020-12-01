#! /usr/bin/env python

import os
import shutil
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge
import cv2


###--------------------------- Global variable ---------------------------###
bridge = CvBridge()

# parameter
save_folder = None
show_flag = None
save_flag = None
serial_port = None
publish_camera_info = None


###--------------------------- Internal funciton ---------------------------###
def GetCameraInfo():
    frame_id = rospy.get_param('~frame_id', 'camera')
    height = rospy.get_param('~height', 0)
    width = rospy.get_param('~width', 0)
    distortion_model = rospy.get_param('~distortion_model', '')
    D = rospy.get_param('~D')
    K = rospy.get_param('~K')
    R = rospy.get_param('~R')
    P = rospy.get_param('~P')
    binning_x = rospy.get_param('~binning_x', 0)
    binning_y = rospy.get_param('~binning_y', 0)
    roi_x_offset = rospy.get_param('~roi/x_offset', 0)
    roi_y_offset = rospy.get_param('~roi/y_offset', 0)
    roi_height = rospy.get_param('~roi/height', 0)
    roi_width = rospy.get_param('~roi/width', 0)
    roi_do_rectify = rospy.get_param<bool>('~roi/do_rectify', False)

    caminfo = CameraInfo()
    caminfo.header.frame_id = frame_id
    caminfo.height = height
    caminfo.width = width
    caminfo.distortion_model = distortion_model
    caminfo.D = D
    caminfo.K = K
    caminfo.R = R
    caminfo.P = P
    caminfo.binning_x = binning_x
    caminfo.binning_y = binning_y
    caminfo.roi.x_offset = roi_x_offset
    caminfo.roi.y_offset = roi_y_offset
    caminfo.roi.height = roi_height
    caminfo.roi.width = roi_width
    caminfo.roi.do_rectify = roi_do_rectify

    return caminfo


def video_capture():
    # Initial node
    rospy.init_node('video_capture', anonymous=True)
    rospy.loginfo('Start the video_capture node')

    # Parameter
    global save_folder, show_flag, save_flag, serial_port, publish_camera_info
    save_folder = rospy.get_param('~save_folder', None)
    show_flag = rospy.get_param('~show_flag', False)
    save_flag = rospy.get_param('~save_flag', False)
    serial_port = rospy.get_param('~serial_port', 0)
    publish_camera_info = rospy.get_param('~publish_camera_info', False)

    # Path
    if save_flag:
        if save_folder is None:
            rospy.logerr('save_folder is None!')
            return
        if os.path.exists(save_folder):
            shutil.rmtree(save_folder)
        os.makedirs(save_folder)

    # Camera parameter
    if publish_camera_info:
        caminfo_msg = GetCameraInfo()
        caminfo_pub = rospy.Publisher('camera_info', CameraInfo, queue_size=1)

    # Prepare to publish image topic
    img_pub = rospy.Publisher('image', Image, queue_size=1)

    # Open videocapture
    rospy.loginfo('serial_port: %s', serial_port)
    cap = cv2.VideoCapture(serial_port)
    if not cap.isOpened():
        rospy.logerr('Read video frame failed!')
        return

    # Read image
    count = 0
    while not rospy.is_shutdown():
        count += 1
        flag, img = cap.read()
        if flag:
            img_msg = bridge.cv2_to_imgmsg(img, 'bgr8')
            time = rospy.Time().now()
            img_msg.header.stamp = time
            img_pub.publish(img_msg)

            if publish_camera_info:
                caminfo_msg.header.stamp = time
                caminfo_pub.publish(caminfo_msg)

            if show_flag:
                cv2.namedWindow('image', 0)
                cv2.imshow('image', img)
                cv2.waitKey(1)

            if save_flag:
                filename = os.path.join(save_folder, '%06d.png' % count)
                cv2.imwrite(filename, img)

    # Exit
    cap.release()


if __name__ == '__main__':

    video_capture()
