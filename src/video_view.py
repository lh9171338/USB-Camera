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
encoding = None


###--------------------------- Internal funciton ---------------------------###
def image_callback(msg):
    rospy.loginfo('Sequence: %d', msg.header.seq)

    # Transform image from ImageConstPtr to cv::Mat
    img = bridge.imgmsg_to_cv2(msg, encoding)

    if show_flag:
        cv2.namedWindow('image', 0)
        cv2.imshow('image', img)
        cv2.waitKey(1)

    if save_flag:
        filename = os.path.join(save_folder, '%06d.png' % msg.header.seq)
        cv2.imwrite(filename, img)


def video_view():
    # Initial node
    rospy.init_node('video_view', anonymous=True)
    rospy.loginfo('Start the video_view node')

    # Parameter
    global save_folder, show_flag, save_flag, encoding
    save_folder = rospy.get_param('~save_folder', None)
    show_flag = rospy.get_param('~show_flag', False)
    save_flag = rospy.get_param('~save_flag', False)
    encoding = rospy.get_param('~encoding', 'bgr8')

    # Path
    if save_flag:
        if save_folder is None:
            rospy.logerr('save_folder is None!')
            return
        if os.path.exists(save_folder):
            shutil.rmtree(save_folder)
        os.makedirs(save_folder)

    # Subscribe image topic
    rospy.Subscriber('image', Image, image_callback, queue_size=10)

    # Loop and wait for callback
    rospy.spin()


if __name__ == '__main__':

    video_view()
