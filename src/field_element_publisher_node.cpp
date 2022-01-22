#include "ros/ros.h"
#include "std_msgs/String.h"

#include <thread>
#include <string>
#include <mutex>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#pragma GCC diagnostic pop

ros::NodeHandle* node;

tf2_ros::TransformBroadcaster * tfBroadcaster;


void publisher_loop(void)
{
    while(ros::ok())
    {
		geometry_msgs::TransformStamped transformStamped;

		transformStamped.header.stamp = ros::Time::now();
		transformStamped.header.frame_id = "base_link";
		transformStamped.child_frame_id = "test_state";

		transformStamped.transform.translation.x = -5;
		transformStamped.transform.translation.y = 0;
		transformStamped.transform.translation.z = 0;

		tf2::Quaternion q;
		q.setRPY(0,0,0);
		transformStamped.transform.rotation.x = q.x();
		transformStamped.transform.rotation.x = q.y();
		transformStamped.transform.rotation.x = q.z();
		transformStamped.transform.rotation.x = q.w();

		tfBroadcaster->sendTransform(transformStamped);
	}
}

int main(int argc, char **argv)
{
	/**
	 * The ros::init() function needs to see argc and argv so that it can perform
	 * any ROS arguments and name remapping that were provided at the command line.
	 * For programmatic remappings you can use a different version of init() which takes
	 * remappings directly, but for most command-line programs, passing argc and argv is
	 * the easiest way to do it.  The third argument to init() is the name of the node.
	 *
	 * You must call one of the versions of ros::init() before using any other
	 * part of the ROS system.
	 */
	ros::init(argc, argv, "field_element_publisher_node");

	ros::NodeHandle n;

	node = &n;

    tfBroadcaster = new tf2_ros::TransformBroadcaster();

    std::thread publisher_thread(publisher_loop);

	ros::spin();
	return 0;
}