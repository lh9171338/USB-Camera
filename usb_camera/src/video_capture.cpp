#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>


using namespace cv;
using namespace std;


//--------------------------- Global variable ---------------------------//
ros::Publisher img_pub;
ros::Publisher caminfo_pub;
sensor_msgs::ImagePtr img_msg;
sensor_msgs::CameraInfo caminfo_msg;

// parameter
string save_folder;
bool show_flag;
bool save_flag;
string serial_port;
bool publish_camera_info;


//--------------------------- Internal funciton ---------------------------//
void GetCameraInfo(ros::NodeHandle nh_priv, sensor_msgs::CameraInfo& cam_info)
{
    string frame_id, distortion_model;
    int height, width, binning_x, binning_y;
    int roi_x_offset, roi_y_offset, roi_height, roi_width;
    vector<double> D, K, R, P;
    bool roi_do_rectify;

    nh_priv.param<string>("frame_id", frame_id, "camera"); 
    nh_priv.param<int>("height", height, 0); 
    nh_priv.param<int>("width", width, 0);    
    nh_priv.param<string>("distortion_model", distortion_model, "");    
    nh_priv.getParam("D", D);
    nh_priv.getParam("K", K);
    nh_priv.getParam("R", R);
    nh_priv.getParam("P", P);
    nh_priv.param<int>("binning_x", binning_x, 0);   
    nh_priv.param<int>("binning_y", binning_y, 0);       
    nh_priv.param<int>("roi/x_offset", roi_x_offset, 0);  
    nh_priv.param<int>("roi/y_offset", roi_y_offset, 0);  
    nh_priv.param<int>("roi/height", roi_height, 0);  
    nh_priv.param<int>("roi/width", roi_width, 0);  
    nh_priv.param<bool>("roi/do_rectify", roi_do_rectify, false);  

    cam_info.header.frame_id = frame_id;
    cam_info.height = height;
    cam_info.width = width;
    cam_info.distortion_model = distortion_model;
    cam_info.D = D;
    memcpy(&cam_info.K[0],  &K[0],  cam_info.K.size() * sizeof(double));
    memcpy(&cam_info.R[0],  &R[0],  cam_info.R.size() * sizeof(double));
    memcpy(&cam_info.P[0],  &P[0],  cam_info.P.size() * sizeof(double));
  
    cam_info.binning_x = binning_x;
    cam_info.binning_y = binning_y;
    cam_info.roi.x_offset = roi_x_offset;
    cam_info.roi.y_offset = roi_y_offset;
    cam_info.roi.height = roi_height;
    cam_info.roi.width = roi_width;
    cam_info.roi.do_rectify = roi_do_rectify;
}


//--------------------------- Main funciton ---------------------------//
int main(int argc, char * argv[])
{
    // Initial node
    ros::init(argc, argv, "video_capture");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    ROS_INFO("Start the video_capture node\n");

    // Parameter
    nh_priv.param<string>("save_folder", save_folder, ""); 
    nh_priv.param<bool>("show_flag", show_flag, false); 
    nh_priv.param<bool>("save_flag", save_flag, false);  
    nh_priv.param<string>("serial_port", serial_port, "/dev/ttyVideo0"); 
    nh_priv.param<bool>("publish_camera_info", publish_camera_info, false);  

    // Camera parameter
    if(publish_camera_info)
    {
        GetCameraInfo(nh_priv, caminfo_msg);
        caminfo_pub = nh.advertise<sensor_msgs::CameraInfo>("camera_info", 1);
    }

    // Prepare to publish image topic
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("image", 1);
    sensor_msgs::ImagePtr msg;

    // Open VideoCapture
    VideoCapture cap(serial_port);
    if(!cap.isOpened())
    {
        ROS_ERROR("Read video frame failed!\n");
        return 0;
    }

    // Read image
    Mat img;
    int count = 0;
    char imgname[128];
    while(ros::ok())
    {
        count++;
        cap >> img;
        if(!img.empty())
        {
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
            msg->header.stamp = ros::Time::now();
            pub.publish(msg);
            if(publish_camera_info)
            {
               caminfo_pub.publish(caminfo_msg); 
            }

            if(show_flag) 
            {
                namedWindow("camera", 0);
                imshow("camera", img);
                waitKey(1);
            }

            if(save_flag)
            {
                sprintf(imgname, "%05d.jpg", count);
                string str(imgname);
                imwrite(save_folder + str, img);
            }
        }

        // ROS_INFO( "Read the %dth frame successfully!", count );
    }

    // Exit
    cap.release();

    return 0;
}
