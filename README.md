[<img height="23" src="https://github.com/lh9171338/Outline/blob/master/icon.jpg"/>](https://github.com/lh9171338/Outline) USB-Camera
===
  
# 1. Introduction
>>This repository is a ROS package including two ROS nodes named video_capture and video_view respectively. The video_capture node can capture USB camera 
video frames and publish them as Image messages. The video_view node can receive Image messages, then show or save the received images.

# 2. Usage
## 2.1 video_capture Node
### 2.1.1 Subscribed Topics  
None

### 2.1.2 Published Topics  
image([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))  
>>Publish the USB camera video frames via this topic.  

camera_info([sensor_msgs/CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html))  
>>Publish the USB camera infomation via this topic.

### 2.1.3 Parameters
~save_folder(string; default: "")  
>>The folder for saving video frames.

~show_flag(bool; default: false)  
>>Whether to show video frames.

~save_flag(bool; defalut: false)  
>>Whether to save video frames. When save_flag is set to true, the video frames will be saved to the folder appointed by save_folder.

~serial_port(string; default: "dev/ttyVideo0")  
>>The serial port of the USB camera.

~publish_camera_info(bool; default: true)  
>>Whether to publish the camera infomation via the camera_info topic.

### 2.1.4 Example  
```
roslaunch usb_camera video_capture.launch
```

## 2.2 video_view Node
### 2.2.1 Subscribed Topics  
image([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))  
>>Receive video frames via this topic. 

### 2.2.2 Published Topics  
None

### 2.2.3 Parameters
~save_folder(string; default: "")  
>>The folder for saving video frames.

~show_flag(bool; default: true)  
>>Whether to show video frames.

~save_flag(bool; defalut: false)  
>>Whether to save video frames. When save_flag is set to true, the video frames will be saved to the folder appointed by save_folder.

~encoding(string; default: "bgr8")  
>>The encode mode of the Image messages.

### 2.2.4 Example  
```
roslaunch usb_camera video_view.launch
```

# 3. Node Graph  
![image](https://github.com/lh9171338/USB-Camera/blob/master/usb_camera/rosgraph.png)

