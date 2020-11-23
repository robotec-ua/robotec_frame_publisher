#include "odometry_publisher/odom_publisher.h"

int main(int argc, char** argv ) {
	//
    ros::init(argc, argv, "odom_publisher");

	//
    Odom odometry;

	//
    ros::spin();


    return 0;
}
