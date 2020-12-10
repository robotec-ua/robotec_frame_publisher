#include <tf2/LinearMath/Matrix3x3.h>

#include "robotec_odometry_publisher/odom_publisher.h"

Odom::Odom():
    linear_velocity_x_(0),
    linear_velocity_y_(0),
    angular_velocity_z_(0),
    last_vel_time_(0),
    vel_dt_(0),
    x_pos_(0),
    y_pos_(0),
    heading_(0)
{
    std::string odom_topic,
                velocity_topic,
                setpose_service;

    // Getting the parameters
    nh_.param<std::string>("odom_topic", odom_topic, "raw_odom");
    nh_.param<std::string>("velocity_topic", velocity_topic, "raw_vel");
    nh_.param<std::string>("setpose_service", setpose_service, "odom/set_pose");

    odom_publisher_ = nh_.advertise<nav_msgs::Odometry>(odom_topic, 50);
    velocity_subscriber_ = nh_.subscribe(velocity_topic, 50, &Odom::velCallback, this);
    set_pose_srv_ = nh_.advertiseService(setpose_service, &Odom::setPoseSrvCallback, this);
}

bool Odom::setPoseSrvCallback(robotec_msgs::SetPose::Request& request,
                              robotec_msgs::SetPose::Response&)
{

    x_pos_ = request.pose.pose.pose.position.x;  // PoseWithCovarianceStamped -> PoseWithCovariance -> Pose
    y_pos_ = request.pose.pose.pose.position.y;
    

    // https://gist.github.com/marcoarruda/f931232fe3490b7fa20dbb38da1195ac
    tf2::Quaternion q(
        request.pose.pose.pose.orientation.x,
        request.pose.pose.pose.orientation.y,
        request.pose.pose.pose.orientation.z,
        request.pose.pose.pose.orientation.w);

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    heading_ = yaw;

    ROS_INFO("Set pose to x: %.6f, y: %.6f, heading: %.6f", x_pos_, y_pos_, heading_);
}

void Odom::velCallback(const robotec_msgs::Velocities& vel) {
    ros::Time current_time = ros::Time::now();

    linear_velocity_x_ = vel.linear_x;
    linear_velocity_y_ = vel.linear_y;
    angular_velocity_z_ = vel.angular_z;

    vel_dt_ = (current_time - last_vel_time_).toSec();
    last_vel_time_ = current_time;

    double delta_heading = angular_velocity_z_ * vel_dt_; //radians
    double delta_x = (linear_velocity_x_ * cos(heading_) - linear_velocity_y_ * sin(heading_)) * vel_dt_; //m
    double delta_y = (linear_velocity_x_ * sin(heading_) + linear_velocity_y_ * cos(heading_)) * vel_dt_; //m

    //calculate current position of the robot
    x_pos_ += delta_x;
    y_pos_ += delta_y;
    heading_ += delta_heading;

    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";

    //robot's position in x,y, and z
    odom.pose.pose.position.x = x_pos_;
    odom.pose.pose.position.y = y_pos_;
    odom.pose.pose.position.z = 0.0;

    //robot's heading in quaternion
    odom.pose.pose.orientation.x = odom_quat.x();
    odom.pose.pose.orientation.y = odom_quat.y();
    odom.pose.pose.orientation.z = odom_quat.z();
    odom.pose.pose.orientation.w = odom_quat.w();
    odom.pose.covariance[0] = 0.001;
    odom.pose.covariance[7] = 0.001;
    odom.pose.covariance[35] = 0.001;

    //linear speed from encoders
    odom.twist.twist.linear.x = linear_velocity_x_;
    odom.twist.twist.linear.y = linear_velocity_y_;
    odom.twist.twist.linear.z = 0.0;

    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;

    //angular speed from encoders
    odom.twist.twist.angular.z = angular_velocity_z_;
    odom.twist.covariance[0] = 0.0001;
    odom.twist.covariance[7] = 0.0001;
    odom.twist.covariance[35] = 0.0001;

    odom_publisher_.publish(odom);
}
