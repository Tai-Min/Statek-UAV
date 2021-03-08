#pragma once

#include <string>
#include <atomic>
#include <mutex>
#include <thread>

#include "opencv2/videoio.hpp"

class Camera {
private:
    std::atomic_bool stopFlag = {false}; //!< Flag used to stop camera thread.
    std::atomic_bool firstFrameRead = {false}; //!< Flag to indicate that first frame was read.
    std::mutex frameMutex; //!< Mutex to block cv::Mat frame.
    std::thread cameraThread; //!< Thread to continuously read frames from the camera.

    cv::VideoCapture cam; //!< Camera object.
    cv::Mat frame; //!< Current frame read from camera.

    /**
     * @brief Start camera thread. Stops until camera thread reads first frame.
     */
    void start();

    /**
     * @brief Stop camera thread.
     */
    void stop();

    /**
     * @brief Function to pass to camera thread.
     */
    void threadFunction();

    /**
     * @brief Get pipeline string.
     * 
     * @param deviceId Camera port select. 0 for CAM0 and 1 for CAM1.
     * @param width Camera's width.
     * @param height Camera's height.
     * @param framerate Camera's framerate.
     * @param flip How to flip camera's result (value between 0 and 3).
     * 
     * @return Pipeline string.
     */
    static std::string pipeline(int deviceId, int width, int height, int framerate, int flip);

public:
    /**
     * @brief Class destructor. Closes opened camera device.
     */
    ~Camera();

    /**
     * @brief Open camera device and start reading.
     * 
     * @param deviceId Camera port select. 0 for CAM0 and 1 for CAM1.
     * @param width Camera's width.
     * @param height Camera's height.
     * @param framerate Camera's framerate.
     * @param flip How to flip camera's result (value between 0 and 3).
     */
    void open(int deviceId, int width, int height, int framerate, int flip);

    /**
     * @brief Close camera device.
     */
    void close();

    /**
     * @brief Check whether camera device is open.
     * 
     * @return True if open.
     */
    bool isOpen();

    /**
     * @brief Get newest frame.
     * 
     * @return Newest frame.
     */
    cv::Mat getFrame();
};