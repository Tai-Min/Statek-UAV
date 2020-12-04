#ifndef STATEK_MOTOR_CONTROL_H
#define STATEK_MOTOR_CONTROL_H

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <std_msgs/Float32.h>

namespace gazebo
{
    /**
    * @brief Plugin to control skid steer like Statek UAV through ROS.
    */
    class MotorControlPlugin : public ModelPlugin
    {
    private:
        event::ConnectionPtr worldUpdateConnection; //!< Callback to send encoder data back to ROS.

        physics::ModelPtr model;           //!< Model to which this plugin is attached to.
        physics::JointPtr leftBackWheel;   //!< Joint of left back wheel.
        physics::JointPtr rightBackWheel;  //!< Joint of right back wheel.
        physics::JointPtr leftFrontWheel;  //!< Joint of left front wheel.
        physics::JointPtr rightFrontWheel; //!< Joint of right front wheel.
        common::PID leftMotorPid;          //!< Pid controller for left motor.
        common::PID rightMotorPid;         //!< Pid controller for right motor.

        std::unique_ptr<ros::NodeHandle> rosNode; //!< Node to communicate with ROS.
        ros::Subscriber leftMotorCmdSubscriber;   //!< Subscriber that listens to <model_name>/left_vel_cmd and sets new velocity target based on it.
        ros::Subscriber rightMotorCmdSubscriber;  //!< Subscriber that listens to <model_name/right_vel_cmd and sets new velocity target based on it.

        ros::Publisher leftMotorRawData;  //!< Publisher that publishes raw info from encoder that placed on left back wheel.
        ros::Publisher rightMotorRawData; //!< Publisher that publishes raw info from encoder that is placed on right back wheel.

        ros::CallbackQueue rosQueue; //!< Callback manager for ROS.
        std::thread rosQueueThread;  //!< Thread for ROS communication.
        std::atomic_bool rosStop = {false};
        
        /**
        * @brief Initialize ROS, subscribers, publishers and start ROS thread.
        */
        void InitializeRosSubscribersPublishers();

        /**
        * @brief Create subscribers required to command both motors.
        */
        void CreateSubscribers();

        /**
        * @brief Start Thread to manage ROS communication.
        */
        void StartRosThread();

        /**
        * @brief Save joint pointers based on joint names from sdf file or default names.
        *
        * @param _sdf Pointer to sdf pointer.
        */
        void saveJoints(sdf::ElementPtr _sdf);

        /**
        * @brief Save pid controllers with gains based on values from sdf file or default gains.
        *
        * @param _sdf Pointer to sdf pointer.
        */
        void SavePids(sdf::ElementPtr _sdf);

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
        * @brief Method invoked when something was published on vel_cmd_left topic.
        *
        * @param _msg Target speed for left side normalized to <-1;1>.
        */
        void OnVelCmdLeft(const std_msgs::Float32ConstPtr &_msg);

        /**
        * @brief Method invoked when something was published on vel_cmd_right topic.
        *
        * @param _msg Target speed for right side normalized to <-1;1>.
        */
        void OnVelCmdRight(const std_msgs::Float32ConstPtr &_msg);

    public:
        /**
        * @brief Class constructor.
        */
        MotorControlPlugin() : ModelPlugin() {}
        ~MotorControlPlugin();

        /**
        * @brief The load function is called by Gazebo when the plugin is inserted into simulation
        *
        * @param _model A pointer to the model that this plugin is attached to.
        * @param _sdf A pointer to the plugin's SDF element.
        */
        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    };

    GZ_REGISTER_MODEL_PLUGIN(MotorControlPlugin)
} // namespace gazebo

#endif