#ifndef ODOM_H
#define ODOM_H

#include <ros/ros.h>
#include <robotec_msgs/Velocities.h>
#include <robotec_msgs/SetPose.h>

#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

class Odom final {
public:
    Odom();
    void velCallback(const robotec_msgs::Velocities& vel);
    bool setPoseSrvCallback(robotec_msgs::SetPose::Request& request,
                            robotec_msgs::SetPose::Response&);


private:
    ros::NodeHandle nh_;
    ros::Publisher odom_publisher_;
    ros::Subscriber velocity_subscriber_;
    tf2::Quaternion odom_quat;
    nav_msgs::Odometry odom;
    ros::ServiceServer set_pose_srv_;

    // float steering_angle_;
    float linear_velocity_x_;
    float linear_velocity_y_;
    float angular_velocity_z_;
    ros::Time last_vel_time_;
    float vel_dt_;
    float x_pos_;
    float y_pos_;
    float heading_;
};

#endif
