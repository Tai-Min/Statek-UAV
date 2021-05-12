#include "../../include/vision_publisher/camera.hpp"
#include <sstream>
#include <chrono>

Camera::~Camera()
{
    try {
        this->close();
    }
    catch(...) {

    }
}

void Camera::start()
{
    this->stopFlag = false;
    this->firstFrameRead = false;

    this->cameraThread = std::thread(&Camera::threadFunction, this);

    // wait for the first frame
    while(!this->firstFrameRead){
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void Camera::stop()
{
    this->stopFlag = true;
    this->cameraThread.join();
}

void Camera::threadFunction()
{
    while(true){
        if(this->stopFlag)
            return;

        // get frame
        {
            std::unique_lock<std::mutex> lck(this->frameMutex);
            this->cam.read(this->frame);
            if(!this->frame.empty())
                this->firstFrameRead = true;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

std::string Camera::pipeline(int deviceId, int width, int height, int framerate, int flip)
{
    std::stringstream ss;
    ss << "nvarguscamerasrc sensor_id=" << deviceId << " ! " 
    << "video/x-raw(memory:NVMM), "
    << "width=(int)" << width << ", height=" << height << ", "
    << "format=(string)NV12, framerate=(fraction)" << framerate << "/1 ! "
    << "nvvidconv flip-method=" << flip << " ! "
    << "video/x-raw, width=" << width << ", height=(int)" << height << ", format=(string)BGRx ! "
    << "videoconvert ! "
    << "video/x-raw, format=(string)BGR ! appsink";

    return ss.str();
}

void Camera::open(int deviceId, int width, int height, int framerate, int flip)
{
    // try to close camera in case it's already opened
    try {
        this->close();
    }
    catch(...) {

    }

    std::string p = this->pipeline(deviceId, width, height, framerate, flip);
    this->cam = cv::VideoCapture(p, cv::CAP_GSTREAMER);

    if(this->cam.isOpened())
        this->start();
}

void Camera::close()
{
    this->stop();
    this->cam.release();
}

bool Camera::isOpen()
{
    return this->cam.isOpened();
}

cv::Mat Camera::getFrame()
{
    std::unique_lock<std::mutex> lck(this->frameMutex);
    return this->frame.clone();
}