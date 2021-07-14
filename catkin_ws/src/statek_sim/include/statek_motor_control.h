#ifndef STATEK_MOTOR_CONTROL_H
#define STATEK_MOTOR_CONTROL_H

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <statek_hw/Velocity.h>

#include <tf2_ros/transform_broadcaster.h>
#include "odometry.hpp"

namespace gazebo
{
    /**
    * @brief Plugin to control skid steer like Statek UAV through ROS.
    */
    class MotorControlPlugin : public ModelPlugin
    {
    private:
        Odometry odomUpdater;

        event::ConnectionPtr worldUpdateConnection; //!< Callback to send encoder data back to ROS.

        physics::ModelPtr model;           //!< Model to which this plugin is attached to.
        physics::JointPtr leftBackWheel;   //!< Joint of left back wheel.
        physics::JointPtr rightBackWheel;  //!< Joint of right back wheel.
        physics::JointPtr leftFrontWheel;  //!< Joint of left front wheel.
        physics::JointPtr rightFrontWheel; //!< Joint of right front wheel.

        common::PID leftBackWheelPid;   //!< Pid controller for left back wheel.
        common::PID rightBackWheelPid;  //!< Pid controller for right back wheel.
        common::PID leftFrontWheelPid;  //!< Pid controller for left front wheel.
        common::PID rightFrontWheelPid; //!< Pid controller for right front wheel.

        std::unique_ptr<ros::NodeHandle> rosNode; //!< Node to communicate with ROS.
        ros::Rate rosLoopRate;
        ros::Subscriber motorCmdSubscriber;       //!< Subscriber that listens to <model_name>/left_vel_cmd and sets new velocity target based on it.
        ros::Publisher leftMotorRawData;          //!< Publisher that publishes raw info from encoder that placed on left back wheel.
        ros::Publisher rightMotorRawData;         //!< Publisher that publishes raw info from encoder that is placed on right back wheel.
        ros::Publisher odomPublisher;             //!< Publisher that publishes odometry from encoders.
        tf2_ros::TransformBroadcaster odomBroadcaster; //!< Odometry tf broadcaster.
        std::random_device rd;
        std::mt19937 randomGen;
        std::normal_distribution<> noise;

        ros::CallbackQueue rosQueue;        //!< Callback manager for ROS.
        std::thread rosQueueThread;         //!< Thread for ROS communication.
        std::atomic_bool rosStop = {false}; //!< Flag to stop ROS thread.

        std::atomic<double> leftTarget = {0};    //!< Setpoint for left wheels.
        std::atomic<double> rightTarget = {0};   //!< Setpoint for right wheels.
        std::atomic<double> leftVelocity = {0};  //!< Angular velocity of left wheels.
        std::atomic<double> rightVelocity = {0}; //!< Angular velocity of right wheels.
        std::atomic<double> leftPosition = {0};  //!< Angular position of left wheels.
        std::atomic<double> rightPosition = {0}; //!< Angular position of right wheels.

        std::string left_motor_tf = "left_motor_link";      //!< For Encoder's header.
        std::string right_motor_tf = "right_motor_link";    //!< For Encoder's header.
        std::string odom_tf_frame = "odom/odom_link";       //!< For Odom's header.
        std::string odom_tf_child_frame = "base_footprint"; //!< For Odom's header.

        double maxRpmRads = 0;

        /**
         * @brief Initialize odometry updater.
         */
        void InitializeOdom(sdf::ElementPtr _sdf);

        /**
        * @brief Initialize ROS, subscribers, publishers and start ROS thread.
        */
        void InitializeRos(sdf::ElementPtr _sdf);

        /**
        * @brief Create subscribers required to command both motors.
        */
        void CreateSubscribers(sdf::ElementPtr _sdf);

        /**
         * @brieg Create publishers to send raw and filtered encoder data.
         */
        void CreatePublishers(sdf::ElementPtr _sdf);

        /**
        * @brief Start Thread to manage ROS communication.
        */
        void StartRosThread();

        /**
         * @brief Create additive gaussian noise generator.
         * 
         * @param _sdf Pointer to sdf of the robot.
         */
        void CreateNoiseGenerator(sdf::ElementPtr _sdf);

        /**
         * @brief Generate gaussian noise sample using noise generator.
         */
        double GenerateNoiseSample();

        /**
        * @brief Save joint pointers based on joint names from sdf file or default names.
        *
        * @param _sdf Pointer to sdf of the robot.
        */
        void SaveJoints(sdf::ElementPtr _sdf);

        /**
        * @brief Save pid controllers with gains based on values from sdf file or default gains.
        *
        * @param _sdf Pointer to sdf of the robot.
        */
        void CreatePids(sdf::ElementPtr _sdf);

        /**
        * Set pids to joints and set target velocities to 0.
        */
        void AttachPidsToJoints();

        /**
        * @brief Callback invoked on Gazebo's world update.
        */
        void OnWorldUpdate();

        /**
        * @brief Method invoked as separate thread to manage communication with ROS.
        */
        void RosQueueThread();

        /**
        * @brief Method invoked when something was published on vel_cmd topic.
        *
        * @param _msg Target speed for wheels in rad/s.
        */
        void OnVelCmd(const statek_hw::Velocity::ConstPtr &_msg);

    public:
        /**
        * @brief Class constructor.
        */
        MotorControlPlugin() : ModelPlugin(), rosLoopRate(10) {}
        ~MotorControlPlugin();

        /**
        * @brief The load function is called by Gazebo when the plugin is inserted into simulation
        *
        * @param _model A pointer to the model that this plugin is attached to.
        * @param _sdf Pointer to sdf of the robot.
        */
        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    };

    GZ_REGISTER_MODEL_PLUGIN(MotorControlPlugin)
} // namespace gazebo

#endif