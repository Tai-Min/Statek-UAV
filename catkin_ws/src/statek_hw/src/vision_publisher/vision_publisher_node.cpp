#include <string>

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include "../../include/vision_publisher/ros_camera.hpp"

int main(int argc, char **argv){
    ros::init(argc, argv, "vision_publisher");

    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);

    // load params
    std::string statekName;
    int cameraWidth;
    int cameraHeight;
    int cameraFramerate;
    int cameraFlip;

    nh.param<std::string>("statek_name", statekName, "statek");
    nh.param<int>(statekName + "/camera_config/width", cameraWidth, 480);
    nh.param<int>(statekName + "/camera_config/height", cameraHeight, 270);
    nh.param<int>(statekName + "/camera_config/framerate", cameraFramerate, 15);
    nh.param<int>(statekName + "/camera_config/flip", cameraFlip, 0);

    RosCamera cam_center(nh, it, statekName, "left", cameraWidth, cameraHeight, cameraFramerate, cameraFlip);
    RosCamera cam_right(nh, it, statekName, "right", cameraWidth, cameraHeight, cameraFramerate, cameraFlip);    

    ros::Rate rate = ros::Rate(cameraFramerate);
    while(ros::ok()){
        ros::Time stamp = ros::Time::now();

        cam_center.publish(stamp);
        cam_right.publish(stamp);

        ros::spinOnce();
        rate.sleep();
    }
}