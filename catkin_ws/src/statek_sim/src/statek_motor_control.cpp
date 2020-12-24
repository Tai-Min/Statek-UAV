#include "../include/statek_motor_control.h"

#include <math.h>

#define MAX_RPM 50
#define MAX_RPM_IN_RAD 2 * M_PI / 60 * MAX_RPM

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

    // prepare joints
    saveJoints(_sdf);

    // prepare pid regulators
    SavePids(_sdf);
    //AttachPidsToJoints();

    // initialize ROS
    InitializeRosSubscribersPublishers();
  }

  void MotorControlPlugin::InitializeRosSubscribersPublishers()
  {
    if (!ros::isInitialized())
    {
      int argc = 0;
      char **argv = NULL;
      ros::init(argc, argv, "gazebo_client",
                ros::init_options::NoSigintHandler);
    }

    this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

    CreateSubscribers();
    StartRosThread();
  }

  void MotorControlPlugin::CreateSubscribers()
  {
    ros::SubscribeOptions leftMotorOptions =
        ros::SubscribeOptions::create<std_msgs::Float32>(
            "/" + this->model->GetName() + "/vel_cmd_left",
            10,
            boost::bind(&MotorControlPlugin::OnVelCmdLeft, this, _1),
            ros::VoidPtr(), &this->rosQueue);
    this->leftMotorCmdSubscriber = this->rosNode->subscribe(leftMotorOptions);

    ros::SubscribeOptions rightMotorOptions =
        ros::SubscribeOptions::create<std_msgs::Float32>(
            "/" + this->model->GetName() + "/vel_cmd_right",
            10,
            boost::bind(&MotorControlPlugin::OnVelCmdRight, this, _1),
            ros::VoidPtr(), &this->rosQueue);
    this->rightMotorCmdSubscriber = this->rosNode->subscribe(rightMotorOptions);
  }

  void MotorControlPlugin::StartRosThread()
  {
    this->rosQueueThread =
        std::thread(std::bind(&MotorControlPlugin::RosQueueThread, this));
  }

  void MotorControlPlugin::saveJoints(sdf::ElementPtr _sdf)
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

  void MotorControlPlugin::SavePids(sdf::ElementPtr _sdf)
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

    this->leftMotorPid = common::PID(kp, ki, kd, MAX_RPM_IN_RAD, -MAX_RPM_IN_RAD, MAX_RPM_IN_RAD, -MAX_RPM_IN_RAD);
    this->rightMotorPid = common::PID(kp, ki, kd, MAX_RPM_IN_RAD, -MAX_RPM_IN_RAD, MAX_RPM_IN_RAD, -MAX_RPM_IN_RAD);
  }

  void MotorControlPlugin::AttachPidsToJoints()
  {
    this->model->GetJointController()->SetVelocityPID(
        this->leftBackWheel->GetScopedName(), this->leftMotorPid);
    this->model->GetJointController()->SetVelocityPID(
        this->rightBackWheel->GetScopedName(), this->rightMotorPid);

    this->model->GetJointController()->SetVelocityTarget(
        this->leftBackWheel->GetScopedName(), 0);
    this->model->GetJointController()->SetVelocityTarget(
        this->rightBackWheel->GetScopedName(), 0);
  }

  void MotorControlPlugin::OnWorldUpdate()
  {

    this->leftBackWheel->SetVelocity(0, this->leftTarget);
    this->rightBackWheel->SetVelocity(0, this->rightTarget);
  
    this->leftFrontWheel->SetVelocity(0, this->leftTarget);
    this->rightFrontWheel->SetVelocity(0, this->rightTarget);
  }

  void MotorControlPlugin::RosQueueThread()
  {
    static const double timeout = 0.01;
    while (this->rosNode->ok() && !this->rosStop)
    {
      this->rosQueue.callAvailable(ros::WallDuration(timeout));
    }
  }

  void MotorControlPlugin::OnVelCmdLeft(const std_msgs::Float32ConstPtr &_msg)
  {
    /*this->model->GetJointController()->SetVelocityTarget(
        this->leftBackWheel->GetScopedName(), -1 * _msg->data * MAX_RPM_IN_RAD);*/
    //this->leftBackWheel->SetVelocity(0, -1 * _msg->data * MAX_RPM_IN_RAD);
    //this->leftFrontWheel->SetVelocity(0, -1 * _msg->data * MAX_RPM_IN_RAD);
    this->leftTarget = {-1 * _msg->data * MAX_RPM_IN_RAD};
  }

  void MotorControlPlugin::OnVelCmdRight(const std_msgs::Float32ConstPtr &_msg)
  {
    /*this->model->GetJointController()->SetVelocityTarget(
        this->rightBackWheel->GetScopedName(), _msg->data * MAX_RPM_IN_RAD);*/
    //this->rightBackWheel->SetVelocity(0, _msg->data * MAX_RPM_IN_RAD);
    //this->rightFrontWheel->SetVelocity(0, _msg->data * MAX_RPM_IN_RAD);
    this->rightTarget = {_msg->data * MAX_RPM_IN_RAD};
  }

} // namespace gazebo