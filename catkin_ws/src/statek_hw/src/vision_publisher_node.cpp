#include <string>

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include "../include/ros_camera.hpp"

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
    std::string configNamespace = statekName + "/camera_config/";
    nh.param<int>(configNamespace + "width", cameraWidth, 480);
    nh.param<int>(configNamespace + "height", cameraHeight, 270);
    nh.param<int>(configNamespace + "framerate", cameraFramerate, 15);
    nh.param<int>(configNamespace + "flip", cameraFlip, 0);

    ros::Rate rate = ros::Rate(cameraFramerate);

    RosCamera cam_center(nh, it, statekName, "left", cameraWidth, cameraHeight, cameraFramerate, cameraFlip);
    RosCamera cam_right(nh, it, statekName, "right", cameraWidth, cameraHeight, cameraFramerate, cameraFlip);    

    while(ros::ok()){
        ros::Time stamp = ros::Time::now();

        cam_center.publish(stamp);
        cam_right.publish(stamp);

        ros::spinOnce();
        rate.sleep();
    }
}