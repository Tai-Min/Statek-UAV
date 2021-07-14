#include <ros/ros.h>
#include "../include/mpc.hpp"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include <tf2/LinearMath/Quaternion.h>

namespace
{
    geometry_msgs::PoseArray getPoseArray(const MPC::Control &control)
    {
        geometry_msgs::PoseArray poses;

        for (auto state : control.stateTrajectory)
        {
            geometry_msgs::Pose pose;
            pose.position.x = state.x;
            pose.position.y = state.y;

            tf2::Quaternion q;
            q.setRPY(0, 0, state.yaw);
            pose.orientation.w = q.w();
            pose.orientation.x = q.x();
            pose.orientation.y = q.y();
            pose.orientation.z = q.z();

            poses.poses.push_back(pose);
        }

        poses.header.frame_id = "statek/map/local_map_link";
        poses.header.stamp = ros::Time::now();

        return poses;
    }
}

int main(int argc, char **argv)
{
    // ROS init.
    ros::init(argc, argv, "short_term_navigator");
    ros::NodeHandle nh("~");

    // Get all the params.
    std::string statekName, pathTopic, twistCallbackTopic, poseTopic, twistTopic;
    double horizonDuration, horizonSampling;
    double linearMax, angularMax;
    MPC::CostWeights weights;

    nh.param<std::string>("statek_name", statekName, "statek");
    nh.param<std::string>("path_topic", pathTopic, "/" + statekName + "/short_term_path");
    nh.param<std::string>("twist_feedback_topic", twistCallbackTopic, "/" + statekName + "/real_time/motors/twist_feedback");
    nh.param<std::string>("pose_topic", poseTopic, "/" + statekName + "/short_term_mpc_poses");
    nh.param<std::string>("twist_topic", twistTopic, "/" + statekName + "/real_time/motors/twist_cmd");

    nh.param<double>("horizon_duration", horizonDuration, 5);    // In seconds.
    nh.param<double>("horizon_sampling", horizonSampling, 0.25); // In seconds.

    nh.param<double>("max_linear_velocity", linearMax, 0.247);    // In m/s.
    nh.param<double>("max_angular_velocity", angularMax, 0.8033); // In rad/s.

    // Cost weights.
    nh.param<double>("pose_weight", weights.poseWeight, 10);
    nh.param<double>("cte_weight", weights.cteWeight, 10);
    nh.param<double>("oe_weight", weights.oeWeight, 700);

    // Actuator use weights.
    nh.param<double>("linear_velocity_weight", weights.linearVelocityWeight, 1);
    nh.param<double>("angular_velocity_weight", weights.angularVelocityWeight, 50);

    // Smoothness of movement weights.
    nh.param<double>("linear_velocity_delta_weight", weights.linearVelocityDeltaWeight, 1);
    nh.param<double>("angular_velocity_delta_weight", weights.angularVelocityDeltaWeight, 50);

    // Initialize MPC assuming that it can't move backwards.
    MPC mpc(horizonDuration, horizonSampling, {linearMax * 0.5, linearMax, -angularMax, angularMax}, weights);

    // Init subscribers and publishers.
    ros::Subscriber pathSub = nh.subscribe(pathTopic, 1, &MPC::onNewPath, &mpc);
    ros::Subscriber twistSub = nh.subscribe(twistCallbackTopic, 1, &MPC::onNewInputs, &mpc);
    ros::Publisher posePublisher = nh.advertise<geometry_msgs::PoseArray>(poseTopic, 1);
    ros::Publisher twistPublisher = nh.advertise<geometry_msgs::Twist>(twistTopic, 1);

    // The main loop.
    ros::Rate rate = ros::Rate(10);
    while (ros::ok())
    {
        MPC::Control control = mpc.update();
        if (control.success)
        {
            // Publish twist command.
            geometry_msgs::Twist twistCmd;
            twistCmd.linear.x = control.linearVelocity;
            twistCmd.angular.z = control.angularVelocity;
            //twistPublisher.publish(twistCmd);

            // Publish array of predicted poses.
            posePublisher.publish(getPoseArray(control));
        }

        ros::spinOnce();
        rate.sleep();
    }
}