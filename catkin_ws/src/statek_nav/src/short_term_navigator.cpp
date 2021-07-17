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
            pose.position.z = 0.1; // A bit above path.

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
    std::string statekName, pathTopic, twistCallbackTopic, poseTopic, twistTopic, solverOptions;
    double horizonDurationForward, horizonSamplingForward, horizonDurationRotate, horizonSamplingRotate;
    double linearMax, angularMax;
    double goalArea;
    MPC::CostWeights weights;

    nh.param<std::string>("statek_name", statekName, "statek");
    nh.param<std::string>("path_topic", pathTopic, "/" + statekName + "/short_term_path");
    nh.param<std::string>("twist_feedback_topic", twistCallbackTopic, "/" + statekName + "/real_time/motors/twist_feedback");
    nh.param<std::string>("pose_topic", poseTopic, "/" + statekName + "/short_term_mpc_poses");
    nh.param<std::string>("twist_topic", twistTopic, "/" + statekName + "/real_time/motors/twist_cmd");
    nh.param<std::string>("solver_options", solverOptions, "");

    nh.param<double>("goal_area", goalArea, 0.25);

    nh.param<double>("horizon_duration_forward", horizonDurationForward, 40);  // In seconds.
    nh.param<double>("horizon_sampling_forward", horizonSamplingForward, 1.5); // In seconds.
    nh.param<double>("horizon_duration_rotate", horizonDurationRotate, 8);     // In seconds.
    nh.param<double>("horizon_sampling_rotate", horizonSamplingRotate, 0.5);   // In seconds.

    nh.param<double>("max_linear_velocity", linearMax, 0.247);    // In m/s.
    nh.param<double>("max_angular_velocity", angularMax, 0.8033); // In rad/s.

    // Cost weights.
    nh.param<double>("pose_weight", weights.poseWeight, 5);
    nh.param<double>("cte_weight", weights.cteWeight, 500);
    nh.param<double>("oe_weight", weights.oeWeight, 500);

    // Actuator use weights.
    nh.param<double>("linear_velocity_weight", weights.linearVelocityWeight, 1);
    nh.param<double>("angular_velocity_weight", weights.angularVelocityWeight, 1);

    // Smoothness of movement weights.
    nh.param<double>("linear_velocity_delta_weight", weights.linearVelocityDeltaWeight, 3);
    nh.param<double>("angular_velocity_delta_weight", weights.angularVelocityDeltaWeight, 3);

    // Initialize MPC assuming that it can't move backwards.
    MPC mpc(horizonDurationForward, horizonSamplingForward,
            horizonDurationRotate, horizonSamplingRotate,
            {0, linearMax, -1 * angularMax, angularMax},
            weights, goalArea, solverOptions);

    // Init subscribers and publishers.
    ros::Subscriber pathSub = nh.subscribe(pathTopic, 1, &MPC::onNewPath, &mpc);
    ros::Subscriber twistSub = nh.subscribe(twistCallbackTopic, 1, &MPC::onNewInputs, &mpc);
    ros::Publisher posePublisher = nh.advertise<geometry_msgs::PoseArray>(poseTopic, 1);
    ros::Publisher twistPublisher = nh.advertise<geometry_msgs::Twist>(twistTopic, 1);

    // The main loop.
    ros::Rate rate = ros::Rate(50);
    while (ros::ok())
    {
        MPC::Control control = mpc.update();

        // Publish twist command.
        geometry_msgs::Twist twistCmd;
        twistCmd.linear.x = control.linearVelocity;
        twistCmd.angular.z = control.angularVelocity;
        twistPublisher.publish(twistCmd);

        // Publish array of predicted poses.
        posePublisher.publish(getPoseArray(control));

        ros::spinOnce();
        rate.sleep();
    }
}