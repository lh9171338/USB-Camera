#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>


using namespace sensor_msgs;
using namespace cv;
using namespace std;


// Global variables
string save_folder;
bool show_flag;
bool save_flag;
char filename[128];
string encoding;
Mat img;


//--------------------------- Callback funciton ---------------------------//
void ImageCallback(const ImageConstPtr& msg)
{
    ROS_INFO("Sequence: %d", msg->header.seq);

    // Transform image from ImageConstPtr to cv::Mat
    cv_bridge::CvImagePtr cv_ptr_image;
    cv_ptr_image = cv_bridge::toCvCopy(msg, encoding);
    img = cv_ptr_image->image;

    if(show_flag)
    {
        namedWindow("iamge", 0);
        imshow("iamge", img);
        waitKey(1);
    }

    if(save_flag)
    {
        sprintf(filename,"%05d.jpg", msg->header.seq);
        std::string str(filename);
        imwrite(save_folder + str, img);
    }
}

int main(int argc,char** argv)
{
    // Initial node
    ros::init(argc,argv,"video_view");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    ROS_INFO("Start the video_view node\n");

    // Parameter
    nh_priv.param<string>("save_folder", save_folder, ""); 
    nh_priv.param<bool>("show_flag", show_flag, true);  
    nh_priv.param<bool>("save_flag", save_flag, false); 
    nh_priv.param<string>("encoding", encoding, "bgr8");   

    // Subscribe image topic
    ros::Subscriber sub = nh.subscribe("image", 10, ImageCallback);

    // Loop and wait for callback
    ros::spin();
}


