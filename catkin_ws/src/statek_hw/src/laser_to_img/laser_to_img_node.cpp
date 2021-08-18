#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include "opencv2/videoio.hpp"

#include "cv_bridge/cv_bridge.h"
#include <iostream>
namespace
{
    bool receivedFirstMsg = false;
    double imgWidthMeters;
    double imgHeightMeters;
    double bresenhamScalingParam;
    int imgWidth;
    int imgHeight;
    sensor_msgs::Image scanImg;
}

double norm(int y, int x){
    return sqrt(pow(y, 2) + pow(x, 2));
}

double distanceFromCenter(int y, int x)
{
    return norm(y - imgHeight / 2, x - imgWidth / 2);
}

void onNewScan(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    receivedFirstMsg = true;

    cv::Mat scanImgMat(imgHeight, imgWidth, CV_8UC1, cv::Scalar::all(0));

    double currentAngle = scan->angle_min;
    double increment = scan->angle_increment;

    std::vector<cv::Point> obstacles;

    // Place obstacles.
    for (unsigned int i = 0; i < scan->ranges.size(); i++)
    {
        double ray = scan->ranges[i];

        // 0 means there is no obstacle detected.
        if (ray == 0)
        {
            currentAngle += increment;
            continue;
        }

        double xMeters = ray * cos(currentAngle);
        double yMeters = ray * sin(currentAngle);

        // Cast to indexes in matrix.
        xMeters = xMeters / (double)(imgWidthMeters / imgWidth) + imgWidth / 2.0;
        yMeters = yMeters / (double)(imgHeightMeters / imgHeight) + imgHeight / 2.0;

        int x = xMeters;
        int y = yMeters;

        // Out of bounds.
        if (x < 0 || y < 0 || x >= imgWidth || y >= imgHeight)
        {
            currentAngle += increment;
            continue;
        }

        std::cout << "ADD" << std::endl;
        scanImgMat.at<uint8_t>(y, x) = 255;
        obstacles.push_back(cv::Point(x, y));

        currentAngle += increment;
    }

    // Fill gaps.
    for (unsigned int i = 0; i < obstacles.size(); i++)
    {
        // Get distance from center to check what maximum distance
        // between two obstacle pixels is allowed.
        double distanceCenter = distanceFromCenter(obstacles[i].y, obstacles[i].x);
        double maxAllowedDistance = bresenhamScalingParam * distanceCenter;

        // Fill space between two close enough obstacles.
        for (unsigned int j = i + 1; j < obstacles.size(); j++)
        {
            double distance = norm(obstacles[i].y - obstacles[j].y, obstacles[i].x - obstacles[j].x);

            if (distance <= maxAllowedDistance){
                cv::line(scanImgMat, obstacles[i], obstacles[j], 255);
            }
                
        }
    }

    cv::rotate(scanImgMat, scanImgMat, cv::ROTATE_90_CLOCKWISE);
    cv::flip(scanImgMat, scanImgMat, 1);

    cv_bridge::CvImage imgBridge = cv_bridge::CvImage(scanImg.header, sensor_msgs::image_encodings::MONO8, scanImgMat);
    imgBridge.toImageMsg(scanImg);

    scanImg.header.stamp = ros::Time::now();
}

int main(int argc, char **argv)
{
    // ROS init.
    ros::init(argc, argv, "vision_publisher");
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);
    sensor_msgs::CameraInfo camInfo;

    // Load params.
    std::string statekName;
    std::string laserImgFrame;
    std::string laserTopic;
    std::string laserImgTopic;
    std::string cameraInfoTopic;

    double bresenhamMaxDistance;
    double bresenhamMaxPixels;

    nh.param<std::string>("statek_name", statekName, "statek");
    nh.param<std::string>("laser_img_frame", laserImgFrame, statekName + "/laser/laser_link");
    nh.param<std::string>("laser_topic", laserTopic, "/" + statekName + "/laser/scan");
    nh.param<std::string>("laser_img_topic", laserImgTopic, "/" + statekName + "/laser/scan_img");
    nh.param<std::string>("laser_img_info_topic", cameraInfoTopic, "/" + statekName + "/laser/camera_info");

    nh.param<double>("width_meters", imgWidthMeters, 6.0);
    nh.param<double>("height_meters", imgHeightMeters, 6.0);
    nh.param<int>("width", imgWidth, 256);
    nh.param<int>("height", imgHeight, 256);
    nh.param<double>("bresenham_max_pixels", bresenhamMaxPixels, 3);
    bresenhamMaxDistance = sqrt(imgWidth * imgWidth + imgHeight * imgHeight);
    bresenhamScalingParam = bresenhamMaxPixels / bresenhamMaxDistance;

    int updateRate;
    nh.param<int>("fps", updateRate, 15);

    // Save some static stuff to the message.
    scanImg.header.frame_id = laserImgFrame;
    scanImg.width = imgWidth;
    scanImg.height = imgHeight;

    // Subscribers and publishers.
    ros::Subscriber laserSub = nh.subscribe(laserTopic, 1, &onNewScan);
    image_transport::Publisher laserImgPublisher = it.advertise(laserImgTopic, 1);

    ros::Rate rate = ros::Rate(updateRate);
    while (ros::ok())
    {
        if (receivedFirstMsg)
        {
            laserImgPublisher.publish(scanImg);
        }

        ros::spinOnce();
        rate.sleep();
    }
}