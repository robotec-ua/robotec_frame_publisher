#include "robotec_frame_publisher/frame_publisher.hpp"

FramePublisher::FramePublisher(ros::NodeHandle handler, ros::NodeHandle private_handler) :
    _nh(handler),
    _private_nh(private_handler)
{
        std::string from_frame,
                twist_topic,
                pose_topic,
                to_frame;

    // Getting the parameters
    _private_nh.param<std::string>("from", from_frame, "odom");
    _private_nh.param<std::string>("to", to_frame, "base_footprint");
    if (_private_nh.hasParam("twist_topic")) {
        _private_nh.param<std::string>("twist_topic", twist_topic);
    }
    else if (_private_nh.hasParam("pose_topic")) {
        _private_nh.param<std::string>("pose_topic", pose_topic, "pose");
    }
    else {
        throw std::invalid_argument("You must specify either Twist or Pose source topic");
    }

    if (twist_topic.empty())
        _subscriber = _nh.subscribe(pose_topic, 50, &FramePublisher::poseCallback, this);
    else 
        _subscriber = _nh.subscribe(twist_topic, 50, &FramePublisher::twistCallback, this);

    // Setting message headers
    _transform.header.frame_id = from_frame;
    _transform.child_frame_id = to_frame;

    //
    _position = {0.0, 0.0, 0.0};

}

void FramePublisher::twistCallback(const geometry_msgs::Twist& twist) {
    float delta_time;
    double delta_heading,
            delta_x,
            delta_y;

    //
    delta_time = (_current_time - _last_time).toSec();
    _current_time = ros::Time::now();
    _last_time = _current_time;

    //
    _velocity[0] = twist.linear.x;
    _velocity[1] = twist.linear.y;
    _velocity[2] = twist.angular.z;

    //
    delta_heading = _velocity[2] * delta_time; //radians
    delta_x = (_velocity[0] * cos(_position[2]) - _velocity[1] * sin(_position[2])) * delta_time; //m
    delta_y = (_velocity[0] * sin(_position[2]) + _velocity[1] * cos(_position[2])) * delta_time; //m

    _position[0] += delta_x;
    _position[1] += delta_y;
    rotation += delta_heading;

    _quaternion.setRPY(0,0, rotation);

    //
    sendTransform();
}

void FramePublisher::poseCallback(const geometry_msgs::Pose& pose) {
    _position[0] = pose.position.x;
    _position[1] = pose.position.y;
    _position[2] = pose.position.z;

    _transform.transform.rotation.x = pose.orientation.x;
    _transform.transform.rotation.y = pose.orientation.y;
    _transform.transform.rotation.z = pose.orientation.z;
    _transform.transform.rotation.w = pose.orientation.w;
}

void FramePublisher::sendTransform(void) {
    _transform.transform.translation.x = _position[0];
    _transform.transform.translation.y = _position[1];
    _transform.transform.translation.z = _position[2];
    _transform.transform.rotation.x = _quaternion.x();
    _transform.transform.rotation.y = _quaternion.y();
    _transform.transform.rotation.z = _quaternion.z();
    _transform.transform.rotation.w = _quaternion.w();

    _transform.header.stamp = _current_time;
    _broadcaster.sendTransform(_transform);
}

int main(int argc, char** argv) {
	//
    ros::init(argc, argv, "sensor_frame_publisher");

    //
    ros::NodeHandle nh,
                    private_nh("~");

	//
    FramePublisher publisher(nh, private_nh);

	//
    ros::spin();


    return 0;
}