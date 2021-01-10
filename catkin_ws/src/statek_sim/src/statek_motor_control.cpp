#include "../include/statek_motor_control.h"
#include "statek_msgs/Encoder.h"
#include <math.h>

namespace gazebo
{
  MotorControlPlugin::~MotorControlPlugin()
  {
    this->rosStop = {true};
    this->rosQueueThread.join();
  }

  void MotorControlPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    // get callback after every world's update
    // to publish encoder's data to ROS
    this->worldUpdateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&MotorControlPlugin::OnWorldUpdate, this));

    this->model = _model;

    // create noise generator for sensors
    CreateNoiseGenerator(_sdf);

    // save joints from sdf
    SaveJoints(_sdf);

    // create pid regulators with correct gains
    CreatePids(_sdf);
    AttachPidsToJoints(); // attach pids to correct joints

    // initialize ROS
    InitializeRos(_sdf);
  }

  void MotorControlPlugin::InitializeRos(sdf::ElementPtr _sdf)
  {
    if (!ros::isInitialized())
    {
      int argc = 0;
      char **argv = NULL;
      ros::init(argc, argv, this->model->GetName() + "_gazebo",
                ros::init_options::NoSigintHandler);
    }

    this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

    int loopRate = 10;

    if (_sdf->HasElement("sensor_publish_rate"))
      loopRate = _sdf->Get<int>("sensor_publish_rate");

    this->rosLoopRate = ros::Rate(loopRate);

    if (_sdf->HasElement("max_rpm"))
      this->maxRpmRads = 2 * M_PI / 60 * _sdf->Get<int>("max_rpm");

    if (_sdf->HasElement("left_motor_tf_frame"))
      this->left_motor_tf = _sdf->Get<std::string>("left_motor_tf_frame");
    if (_sdf->HasElement("right_motor_tf_frame"))
      this->right_motor_tf = _sdf->Get<std::string>("right_motor_tf_frame");

    CreateSubscribers(_sdf);
    CreatePublishers(_sdf);

    StartRosThread();
  }

  void MotorControlPlugin::CreateSubscribers(sdf::ElementPtr _sdf)
  {
    std::string ns = "";
    if (_sdf->HasElement("motor_namespace"))
      ns = _sdf->Get<std::string>("motor_namespace");

    ros::SubscribeOptions motorOptions =
        ros::SubscribeOptions::create<statek_msgs::Velocity>(
            "/" + this->model->GetName() + "/" + ns + "/vel_cmd",
            10,
            boost::bind(&MotorControlPlugin::OnVelCmd, this, _1),
            ros::VoidPtr(), &this->rosQueue);
    this->motorCmdSubscriber = this->rosNode->subscribe(motorOptions);
}

  void MotorControlPlugin::CreatePublishers(sdf::ElementPtr _sdf)
  {
    std::string ns = "";
    if (_sdf->HasElement("sensor_namespace"))
      ns = _sdf->Get<std::string>("sensor_namespace");

    this->leftMotorRawData =
        this->rosNode->advertise<statek_msgs::Encoder>("/" + this->model->GetName() + "/" + ns + "/left/raw", 10);
    this->rightMotorRawData =
        this->rosNode->advertise<statek_msgs::Encoder>("/" + this->model->GetName() + "/" + ns + "/right/raw", 10);
    this->leftMotorFilteredData =
        this->rosNode->advertise<statek_msgs::Encoder>("/" + this->model->GetName() + "/" + ns + "/left/filtered", 10);
    this->rightMotorFilteredData =
        this->rosNode->advertise<statek_msgs::Encoder>("/" + this->model->GetName() + "/" + ns + "/right/filtered", 10);
  }

  void MotorControlPlugin::StartRosThread()
  {
    this->rosQueueThread =
        std::thread(std::bind(&MotorControlPlugin::RosQueueThread, this));
  }

  void MotorControlPlugin::CreateNoiseGenerator(sdf::ElementPtr _sdf)
  {
    double dev = 0;
    double mean = 0;

    if (_sdf->HasElement("sensor_dev"))
      dev = _sdf->Get<double>("sensor_dev");
    if (_sdf->HasElement("sensor_mean"))
      mean = _sdf->Get<double>("sensor_mean");

    this->randomGen = std::mt19937(this->rd());
    this->noise = std::normal_distribution<>(mean, dev);
  }

  double MotorControlPlugin::GenerateNoiseSample()
  {
    return this->noise(this->randomGen);
  }

  void MotorControlPlugin::SaveJoints(sdf::ElementPtr _sdf)
  {
    std::string leftBack = "left_back";
    std::string rightBack = "right_back";
    std::string leftFront = "left_front";
    std::string rightFront = "right_front";

    if (_sdf->HasElement("left_back_joint"))
      leftBack = _sdf->Get<std::string>("left_back_joint");
    if (_sdf->HasElement("right_back_joint"))
      rightBack = _sdf->Get<std::string>("right_back_joint");
    if (_sdf->HasElement("left_front_joint"))
      leftFront = _sdf->Get<std::string>("left_front_joint");
    if (_sdf->HasElement("right_front_joint"))
      rightFront = _sdf->Get<std::string>("right_front_joint");

    this->leftBackWheel = this->model->GetJoint(leftBack);
    this->rightBackWheel = this->model->GetJoint(rightBack);
    this->leftFrontWheel = this->model->GetJoint(leftFront);
    this->rightFrontWheel = this->model->GetJoint(rightFront);
  }

  void MotorControlPlugin::CreatePids(sdf::ElementPtr _sdf)
  {
    double kp = 0.1;
    double ki = 0;
    double kd = 0;

    if (_sdf->HasElement("kp"))
      kp = _sdf->Get<float>("kp");
    if (_sdf->HasElement("ki"))
      ki = _sdf->Get<float>("ki");
    if (_sdf->HasElement("kd"))
      kd = _sdf->Get<float>("kd");

    this->leftBackWheelPid = common::PID(kp, ki, kd);
    this->rightBackWheelPid = common::PID(kp, ki, kd);
    this->leftFrontWheelPid = common::PID(kp, ki, kd);
    this->rightFrontWheelPid = common::PID(kp, ki, kd);
  }

  void MotorControlPlugin::AttachPidsToJoints()
  {
    this->model->GetJointController()->SetVelocityPID(
        this->leftBackWheel->GetScopedName(), this->leftBackWheelPid);
    this->model->GetJointController()->SetVelocityPID(
        this->rightBackWheel->GetScopedName(), this->rightBackWheelPid);

    this->model->GetJointController()->SetVelocityPID(
        this->leftFrontWheel->GetScopedName(), this->leftFrontWheelPid);
    this->model->GetJointController()->SetVelocityPID(
        this->rightFrontWheel->GetScopedName(), this->rightFrontWheelPid);

    this->model->GetJointController()->SetVelocityTarget(
        this->leftBackWheel->GetScopedName(), 0);
    this->model->GetJointController()->SetVelocityTarget(
        this->rightBackWheel->GetScopedName(), 0);

    this->model->GetJointController()->SetVelocityTarget(
        this->leftFrontWheel->GetScopedName(), 0);
    this->model->GetJointController()->SetVelocityTarget(
        this->rightFrontWheel->GetScopedName(), 0);
  }

  void MotorControlPlugin::OnWorldUpdate()
  {
    // update PIDs with target velocities
    this->model->GetJointController()->SetVelocityTarget(
        this->leftBackWheel->GetScopedName(), this->leftTarget);

    this->model->GetJointController()->SetVelocityTarget(
        this->rightBackWheel->GetScopedName(), this->rightTarget);

    this->model->GetJointController()->SetVelocityTarget(
        this->leftFrontWheel->GetScopedName(), this->leftTarget);

    this->model->GetJointController()->SetVelocityTarget(
        this->rightFrontWheel->GetScopedName(), this->rightTarget);

    // save current velocities and positions
    leftVelocity = {this->leftFrontWheel->GetVelocity(0)};
    rightVelocity = {this->rightFrontWheel->GetVelocity(0)};

    if (this->leftFrontWheel->Position(0) < 0)
      leftPosition = {2 * M_PI + fmod(this->leftFrontWheel->Position(0), 2 * M_PI)};
    else
      leftPosition = {fmod(this->leftFrontWheel->Position(0), 2 * M_PI)};

    if (this->rightFrontWheel->Position(0) < 0)
      rightPosition = {2 * M_PI + fmod(this->rightFrontWheel->Position(0), 2 * M_PI)};
    else
      rightPosition = {fmod(this->rightFrontWheel->Position(0), 2 * M_PI)};
  }

  void MotorControlPlugin::RosQueueThread()
  {
    uint32_t cntr = 0;
    while (this->rosNode->ok() && !this->rosStop)
    {
      statek_msgs::Encoder rawLeft;
      rawLeft.header.seq = cntr;
      rawLeft.header.stamp = ros::Time::now();
      rawLeft.header.frame_id = this->left_motor_tf;
      rawLeft.velocity = this->leftVelocity + GenerateNoiseSample();
      rawLeft.position = this->leftPosition + GenerateNoiseSample();

      statek_msgs::Encoder rawRight;
      rawRight.header.seq = cntr;
      rawRight.header.stamp = ros::Time::now();
      rawRight.header.frame_id = this->right_motor_tf;
      rawRight.velocity = this->rightVelocity + GenerateNoiseSample();
      rawRight.position = this->rightPosition + GenerateNoiseSample();

      statek_msgs::Encoder filteredLeft;
      filteredLeft.header.seq = cntr;
      filteredLeft.header.stamp = ros::Time::now();
      filteredLeft.header.frame_id = this->left_motor_tf;
      filteredLeft.velocity = this->leftVelocity;
      filteredLeft.position = this->leftPosition;

      statek_msgs::Encoder filteredRight;
      filteredRight.header.seq = cntr;
      filteredRight.header.stamp = ros::Time::now();
      filteredRight.header.frame_id = this->right_motor_tf;
      filteredRight.velocity = this->rightVelocity;
      filteredRight.position = this->rightPosition;

      this->leftMotorRawData.publish(rawLeft);
      this->rightMotorRawData.publish(rawRight);
      this->leftMotorFilteredData.publish(filteredLeft);
      this->rightMotorFilteredData.publish(filteredRight);

      this->rosQueue.callAvailable();

      cntr++;
      this->rosLoopRate.sleep();
    }
  }

  void MotorControlPlugin::OnVelCmd(const statek_msgs::Velocity::ConstPtr &_msg)
  {
    float left = _msg->left;
    float right = _msg->right;

    // saturation left
    if(left > this->maxRpmRads)
      left = this->maxRpmRads;
    if(left < -this->maxRpmRads)
      left = -this->maxRpmRads;

    // saturation right
    if(right > this->maxRpmRads)
      right = this->maxRpmRads;
    if(right < -this->maxRpmRads)
      right = -this->maxRpmRads;

    this->leftTarget = {left};
    this->rightTarget = {right};
  }

} // namespace gazebo