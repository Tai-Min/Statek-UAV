#pragma once

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/SetCameraInfo.h"
#include "image_transport/image_transport.h"

#include "camera.hpp"

class RosCamera
{
private:
    // For image and info publishers.
    unsigned long long cntr = 0; //!< Sequence counter for header.
    int width;                   //!< Camera's width.
    int height;                  //!< Camera's height.
    std::string position;        //!< Position on robot (either left or right).
    std::string tfLink;          //!< Camera's tf link.

    // To locate camera's parameters in parameter server.
    std::string cameraParamNamespace; //!< Namespace for camera's parameters.

    // ROS stuff.
    ros::NodeHandle &nh;                        //!< Shared ros node handle.
    image_transport::Publisher imgRawPublisher; //!< Image publisher.
    ros::Publisher infoPublisher;               //!< Camera info publisher.
    ros::ServiceServer setInfoService;          //!< Set camera info service.
    sensor_msgs::CameraInfo infoMsg;            //!< Camera info to be published.

    // Hardware drivers.
    Camera camera; //!< Camera device.

    /**
     * @brief Parses camera info into yaml string.
     * 
     * @param info Info to parse.
     * 
     * @return Yaml string.
     */
    std::string getCameraInfoYamlString(const sensor_msgs::CameraInfo &info);

public:
    /**
     * @brief Starts camera device and ROS image / info publishers along with set info service.
     * 
     * @param _nh Shared node handle object.
     * @param it Shared image transport object.
     * @param rosNamespace Namespace on which all the topic should be published.
     * @param position Position on robot frame (either left or right).
     * @param _width Camera's width.
     * @param _height Camera's height.
     * @param framerate Camera's framerate.
     * @param flip How to flip camera's result (value between 0 and 3).
     */
    RosCamera(ros::NodeHandle &_nh, image_transport::ImageTransport &it, std::string rosNamespace, std::string _position, int _width, int _height, int framerate, int flip);

    /**
     * @brief Class destructor. Releases the camera device.
     */
    ~RosCamera();

    /**
     * @brief Get camera info from parameter server.
     * 
     * @return Camera's info from parameter server.
     */
    sensor_msgs::CameraInfo getCameraInfo();

    /**
     * @brief Callback for service set_camera_info.
     * 
     * @return True on success.
     */
    bool setCameraInfo(sensor_msgs::SetCameraInfo::Request &req, sensor_msgs::SetCameraInfo::Response &res);

    /**
     * @brief Publish latest frame with given timestamp.
     * 
     * @param stamp Timestamp to set in message's header.
     */
    void publish(ros::Time stamp);
};