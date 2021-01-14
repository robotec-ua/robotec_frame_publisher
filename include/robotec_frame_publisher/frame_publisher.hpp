#pragma once

#include <ros/ros.h>
#include <array>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

class FramePublisher {
protected:
    ros::NodeHandle _nh,                            // ROS node handler
                    _private_nh;                    // ROS private node handler
    ros::Subscriber _subscriber;                    // ROS subscriber to the topic with Twist or Pose data
    tf2::Quaternion _quaternion;                    // Quaternion of rotation
    geometry_msgs::TransformStamped _transform;
    tf2_ros::TransformBroadcaster _broadcaster;
    ros::Time _last_time,                           // The time of the last publishing
                _current_time;
    float rotation;                                 // The angle of the rotation
    std::array<float,3> _position,                  // The position in 2D
                        _velocity;                  // The velocity in 2D

    /**
     * Sending the transform
     */
    void sendTransform(void);

public:
    /**
     * Constructor
     * @param handler ROS node handler
     * @param private_handler ROS private node handler
     * @throws std::invalid_argument
     */
    FramePublisher(ros::NodeHandle handler, ros::NodeHandle private_handler);

    /**
     * Twist message callback with velocity data
     * @param twist the message with velocities
     */
    void twistCallback(const geometry_msgs::Twist& twist);

    /**
     * Pose message callback with pose data
     * @param pose the carrent pose of the object
     */
    void poseCallback(const geometry_msgs::Pose& pose);
};