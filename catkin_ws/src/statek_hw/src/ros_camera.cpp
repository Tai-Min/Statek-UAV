#include "../include/ros_camera.hpp"

#include <algorithm>
#include <sstream>
#include <fstream>

#include "cv_bridge/cv_bridge.h"
#include "ros/package.h"

RosCamera::RosCamera(ros::NodeHandle &_nh, image_transport::ImageTransport &it, std::string rosNamespace, std::string _position, int _width, int _height, int framerate, int flip)
    : nh(_nh)
{
    this->width = _width;
    this->height = _height;
    this->position = _position;
    this->tfLink = rosNamespace + "/stereo/" + this->position + "_link";
    this->cameraParamNamespace = rosNamespace + "/camera_info_" + this->position + "/";

    std::string imageRawTopic = "/" + rosNamespace + "/stereo/" + this->position + "/image_raw";
    std::string cameraInfoTopic = "/" + rosNamespace + "/stereo/" + this->position + "/camera_info";
    std::string cameraInfoService = "/" + rosNamespace + "/stereo/" + this->position + "/set_camera_info";

    this->imgRawPublisher = it.advertise(imageRawTopic, 1);
    this->infoPublisher = this->nh.advertise<sensor_msgs::CameraInfo>(cameraInfoTopic, 1);
    this->setInfoService = this->nh.advertiseService(cameraInfoService, &RosCamera::setCameraInfo, this);

    this->infoMsg = this->getCameraInfo();

    int deviceId = this->position == "left" ? 0 : 1;
    this->camera.open(deviceId, _width, _height, framerate, flip);
    if (!this->camera.isOpen())
    {
        ROS_WARN("Failed to open /dev/video%d", deviceId);
        exit(-1);
    }
}

RosCamera::~RosCamera()
{
    this->camera.close();
}

std::string RosCamera::getCameraInfoYamlString(const sensor_msgs::CameraInfo &info)
{
    std::stringstream ss;

    ss << "D:\n";
    for (auto d : info.D)
        ss << "- " << d << "\n";

    ss << "K:\n";
    for (auto k : info.K)
        ss << "- " << k << "\n";

    ss << "P:\n";
    for (auto p : info.P)
        ss << "- " << p << "\n";

    ss << "R:\n";
    for (auto r : info.R)
        ss << "- " << r << "\n";

    ss << "distortion_model: " << info.distortion_model;
}

sensor_msgs::CameraInfo RosCamera::getCameraInfo()
{
    sensor_msgs::CameraInfo info;
    info.header.frame_id = this->tfLink;
    info.width = this->width;
    info.height = this->height;
    info.distortion_model = this->nh.param<std::string>(this->cameraParamNamespace + "distortion_model", "plump_bob");

    std::vector<double> D, K, R, P;
    this->nh.param<std::vector<double>>(this->cameraParamNamespace + "D", D, {0, 0, 0, 0, 0});
    this->nh.param<std::vector<double>>(this->cameraParamNamespace + "K", K, {0, 0, 0, 0, 0, 0, 0, 0, 0});
    this->nh.param<std::vector<double>>(this->cameraParamNamespace + "R", R, {0, 0, 0, 0, 0, 0, 0, 0, 0});
    this->nh.param<std::vector<double>>(this->cameraParamNamespace + "P", P, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0});

    for (auto d : D)
        info.D.push_back(d);
    std::copy(K.begin(), K.end(), info.K.begin());
    std::copy(R.begin(), R.end(), info.R.begin());
    std::copy(P.begin(), P.end(), info.P.begin());

    return info;
}

bool RosCamera::setCameraInfo(sensor_msgs::SetCameraInfo::Request &req, sensor_msgs::SetCameraInfo::Response &res)
{
    // local save
    this->infoMsg.distortion_model = req.camera_info.distortion_model;
    std::copy(req.camera_info.D.begin(), req.camera_info.D.end(), this->infoMsg.D.begin());
    std::copy(req.camera_info.K.begin(), req.camera_info.K.end(), this->infoMsg.K.begin());
    std::copy(req.camera_info.R.begin(), req.camera_info.R.end(), this->infoMsg.R.begin());
    std::copy(req.camera_info.P.begin(), req.camera_info.P.end(), this->infoMsg.P.begin());

    // config save.
    std::string yaml = this->getCameraInfoYamlString(req.camera_info);
    std::string path = ros::package::getPath("statek_config") + "/yaml/camera_info_" + this->position + ".yaml";

    try
    {
        std::ofstream f(path);
        f << yaml;
        f.close();
        res.success = true;
        res.status_message = "ok";
    }
    catch (const std::exception &exc)
    {
        res.success = false;
        res.status_message = exc.what();
    }

    return true;
}

void RosCamera::publish(ros::Time stamp)
{
    std_msgs::Header header;
    header.seq = this->cntr;
    header.stamp = stamp;
    header.frame_id = this->tfLink;

    cv::Mat frame = this->camera.getFrame();
    cv_bridge::CvImage imgBridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, frame);

    sensor_msgs::Image frameMsg;
    imgBridge.toImageMsg(frameMsg);

    this->imgRawPublisher.publish(frameMsg);

    this->infoMsg.header.seq = this->cntr;
    this->infoMsg.header.stamp = stamp;

    this->infoPublisher.publish(this->infoMsg);

    this->cntr++;
}