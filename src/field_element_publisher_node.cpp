#include "ros/ros.h"
#include "std_msgs/String.h"

#include <thread>
#include <string>
#include <mutex>

#include <math.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>
#pragma GCC diagnostic pop

#define FEET_TO_METERS 0.3048
#define INCHES_TO_METERS 0.0254
#define DEGREES_TO_RADIANS M_PI / 180.0

static ros::Publisher vis_pub;

ros::NodeHandle* node;

tf2_ros::TransformBroadcaster * tfBroadcaster;

void publish_red_link (void)
{
	geometry_msgs::TransformStamped transformStamped;

	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = "map";
	transformStamped.child_frame_id = "red_link";

	transformStamped.transform.translation.x = 0;
	transformStamped.transform.translation.y = 0;
	transformStamped.transform.translation.z = 0;

	tf2::Quaternion q;
	q.setRPY(0,0,0);
	transformStamped.transform.rotation.x = q.x();
	transformStamped.transform.rotation.y = q.y();
	transformStamped.transform.rotation.z = q.z();
	transformStamped.transform.rotation.w = q.w();

	tfBroadcaster->sendTransform(transformStamped);
}

void publish_blue_link (void)
{
	geometry_msgs::TransformStamped transformStamped;

	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = "map";
	transformStamped.child_frame_id = "blue_link";

	transformStamped.transform.translation.x = 54 * FEET_TO_METERS;
	transformStamped.transform.translation.y = -27 * FEET_TO_METERS;
	transformStamped.transform.translation.z = 0;

	tf2::Quaternion q;
	q.setRPY(0,0,180.0 * DEGREES_TO_RADIANS);
	transformStamped.transform.rotation.x = q.x();
	transformStamped.transform.rotation.y = q.y();
	transformStamped.transform.rotation.z = q.z();
	transformStamped.transform.rotation.w = q.w();

	tfBroadcaster->sendTransform(transformStamped);
}

void publish_hub_link (void)
{
	geometry_msgs::TransformStamped transformStamped;

	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = "red_link";
	transformStamped.child_frame_id = "hub_link";

	transformStamped.transform.translation.x = (54.0 / 2.0) * FEET_TO_METERS;
	transformStamped.transform.translation.y = (-27.0 / 2.0) * FEET_TO_METERS;
	transformStamped.transform.translation.z = 0.0;

	tf2::Quaternion q;
	q.setRPY(0,0,24 * DEGREES_TO_RADIANS);
	transformStamped.transform.rotation.x = q.x();
	transformStamped.transform.rotation.y = q.y();
	transformStamped.transform.rotation.z = q.z();
	transformStamped.transform.rotation.w = q.w();

	tfBroadcaster->sendTransform(transformStamped);
}

void publish_hub_full_height (void)
{
	geometry_msgs::TransformStamped transformStamped;

	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = "hub_link";
	transformStamped.child_frame_id = "hub_full_height";

	transformStamped.transform.translation.x = 0;
	transformStamped.transform.translation.y = 0;
	transformStamped.transform.translation.z = 2.64;

	tf2::Quaternion q;
	q.setRPY(0,0,0);
	transformStamped.transform.rotation.x = q.x();
	transformStamped.transform.rotation.y = q.y();
	transformStamped.transform.rotation.z = q.z();
	transformStamped.transform.rotation.w = q.w();

	tfBroadcaster->sendTransform(transformStamped);
}

void publish_field_perimeter(void)
{
	visualization_msgs::Marker field_perimeter;
	field_perimeter.header.stamp = ros::Time::now();
	field_perimeter.header.frame_id = "red_link";

	field_perimeter.type = visualization_msgs::Marker::LINE_STRIP;
	field_perimeter.action = visualization_msgs::Marker::ADD;
	field_perimeter.id = 0;
	field_perimeter.ns = "FieldWall";
	field_perimeter.color.r = 1;
	field_perimeter.color.g = 1;
	field_perimeter.color.b = 1;
	field_perimeter.color.a = 1;
	field_perimeter.scale.x = .03;

	tf2::Quaternion q;
	q.setRPY(0,0,0);

	field_perimeter.pose.orientation.x = q.x();
	field_perimeter.pose.orientation.y = q.y();
	field_perimeter.pose.orientation.z = q.z();
	field_perimeter.pose.orientation.w = q.w();

	field_perimeter.pose.position.x = 0;
	field_perimeter.pose.position.y = 0;
	field_perimeter.pose.position.z = 0;

	geometry_msgs::Point field_point;
	field_point.x = 0;
	field_point.y = 0;
	field_point.z = 0;
	field_perimeter.points.push_back(field_point);

	field_point.x = 0;
	field_point.y = -252 * INCHES_TO_METERS;
	field_point.z = 0;
	field_perimeter.points.push_back(field_point);

	field_point.x = 69 * INCHES_TO_METERS;
	field_point.y = -27 * FEET_TO_METERS;
	field_point.z = 0;
	field_perimeter.points.push_back(field_point);

	field_point.x = 54 * FEET_TO_METERS;
	field_point.y = -27 * FEET_TO_METERS;
	field_point.z = 0;
	field_perimeter.points.push_back(field_point);

	field_point.x = 54 * FEET_TO_METERS;
	field_point.y = (-27 * FEET_TO_METERS) + (252 * INCHES_TO_METERS);
	field_point.z = 0;
	field_perimeter.points.push_back(field_point);

	field_point.x = (54 * FEET_TO_METERS) - (69 * INCHES_TO_METERS);
	field_point.y = 0;
	field_point.z = 0;
	field_perimeter.points.push_back(field_point);

	field_point.x = 0;
	field_point.y = 0;
	field_point.z = 0;
	field_perimeter.points.push_back(field_point);

	vis_pub.publish(field_perimeter);
}

void publish_field_centerline()
{

	visualization_msgs::Marker center_line;
	center_line.header.stamp = ros::Time::now();
	center_line.header.frame_id = "hub_link";

	center_line.type = visualization_msgs::Marker::LINE_STRIP;
	center_line.action = visualization_msgs::Marker::ADD;
	center_line.id = 0;
	center_line.ns = "Centerline";
	center_line.color.r = 1;
	center_line.color.g = 1;
	center_line.color.b = 1;
	center_line.color.a = 1;
	center_line.scale.x = .03;

	tf2::Quaternion q;
	q.setRPY(0,0,0);

	center_line.pose.orientation.x = q.x();
	center_line.pose.orientation.y = q.y();
	center_line.pose.orientation.z = q.z();
	center_line.pose.orientation.w = q.w();

	float center_line_half_court_length = ((27.0 / 2.0) / cos(24 * DEGREES_TO_RADIANS)) * FEET_TO_METERS;

	center_line.pose.position.x = 0;
	center_line.pose.position.y = 0;
	center_line.pose.position.z = 0;

	geometry_msgs::Point field_point;
	field_point.x = 0;
	field_point.y = center_line_half_court_length;
	field_point.z = 0;
	center_line.points.push_back(field_point);

	field_point.x = 0;
	field_point.y = -center_line_half_court_length;
	field_point.z = 0;
	center_line.points.push_back(field_point);

	vis_pub.publish(center_line);
}

void publish_hub_cylinder(void)
{
	visualization_msgs::Marker hub;
	hub.header.frame_id = "hub_link";
	hub.header.stamp = ros::Time::now();
	hub.ns = "Hub";;
	hub.id = 0;
	hub.type = visualization_msgs::Marker::CYLINDER;
	hub.action = visualization_msgs::Marker::ADD;

	hub.pose.position.x = 0;
	hub.pose.position.y = 0;
	hub.pose.position.z = 2.64 / 2.0;

	tf2::Quaternion q;
	q.setRPY(0,0,0);

	hub.pose.orientation.x = q.x();
	hub.pose.orientation.y = q.y();
	hub.pose.orientation.z = q.z();
	hub.pose.orientation.w = q.w();

	hub.scale.x = 4.0 * FEET_TO_METERS;
	hub.scale.y = 4.0 * FEET_TO_METERS;
	hub.scale.z = 2.64;
	
	hub.color.r = 0.7;
	hub.color.g = 0.7;
	hub.color.b = 0.7;
	hub.color.a = 1.0;

	vis_pub.publish(hub);
}

void publish_hub_base(void)
{
	visualization_msgs::Marker hub;
	hub.header.frame_id = "hub_link";
	hub.header.stamp = ros::Time::now();
	hub.ns = "Hub";;
	hub.id = 5;
	hub.type = visualization_msgs::Marker::CUBE;
	hub.action = visualization_msgs::Marker::ADD;

	hub.pose.position.x = 0;
	hub.pose.position.y = 0;
	hub.pose.position.z = 2.64 / 2.0;

	tf2::Quaternion q;
	q.setRPY(0,0,0);

	hub.pose.orientation.x = q.x();
	hub.pose.orientation.y = q.y();
	hub.pose.orientation.z = q.z();
	hub.pose.orientation.w = q.w();

	hub.scale.x = 4.0 * FEET_TO_METERS;
	hub.scale.y = 4.0 * FEET_TO_METERS;
	hub.scale.z = 2.64;
	
	hub.color.r = 0.7;
	hub.color.g = 0.7;
	hub.color.b = 0.7;
	hub.color.a = 1.0;

	vis_pub.publish(hub);
}

geometry_msgs::Point compute_offset(geometry_msgs::Point base, float angle, float distance)
{
	geometry_msgs::Point result = base;

	result.x += cos(angle) * distance;
	result.y += -sin(angle) * distance;

	return result;
}

void publish_tarmac_lines(int base_id, std::string base_link, bool color_is_red)
{
	visualization_msgs::Marker center_line;
	center_line.header.stamp = ros::Time::now();
	center_line.header.frame_id = base_link;

	center_line.type = visualization_msgs::Marker::LINE_STRIP;
	center_line.action = visualization_msgs::Marker::ADD;
	center_line.id = base_id + 1;
	center_line.ns = "Hub";
	if(color_is_red)
	{
		center_line.color.r = 1;
		center_line.color.g = 0;
		center_line.color.b = 0;
	}
	else
	{
		center_line.color.r = 0;
		center_line.color.g = 0;
		center_line.color.b = 1;
	}
	center_line.color.a = 1;
	center_line.scale.x = .03;

	tf2::Quaternion q;
	q.setRPY(0,0,0);

	center_line.pose.orientation.x = q.x();
	center_line.pose.orientation.y = q.y();
	center_line.pose.orientation.z = q.z();
	center_line.pose.orientation.w = q.w();

	center_line.pose.position.x = 0;
	center_line.pose.position.y = 0;
	center_line.pose.position.z = 0;

	geometry_msgs::Point field_point;
	field_point.x = 0;
	field_point.y = 0;
	field_point.z = 0;
	center_line.points.push_back(field_point);

	field_point = compute_offset(field_point, 90 * DEGREES_TO_RADIANS, (45.23 * INCHES_TO_METERS) / 2.0);
	center_line.points.push_back(field_point);

	field_point = compute_offset(field_point, 45 * DEGREES_TO_RADIANS, 75.07 * INCHES_TO_METERS);
	center_line.points.push_back(field_point);

	field_point = compute_offset(field_point, (-90 + 22.5) * DEGREES_TO_RADIANS, 82.81 * INCHES_TO_METERS);
	center_line.points.push_back(field_point);

	field_point = compute_offset(field_point, (-90 - 22.5) * DEGREES_TO_RADIANS, 82.81 * INCHES_TO_METERS);
	center_line.points.push_back(field_point);

	field_point = compute_offset(field_point, (180 - 45) * DEGREES_TO_RADIANS, 75.07 * INCHES_TO_METERS);
	center_line.points.push_back(field_point);

	field_point = compute_offset(field_point, 90 * DEGREES_TO_RADIANS, (45.23 * INCHES_TO_METERS) / 2.0);
	center_line.points.push_back(field_point);

	vis_pub.publish(center_line);
}

void publish_tarmac_links()
{

	for(int i = 0; i < 4; i++)
	{
		geometry_msgs::TransformStamped transformStamped;

		transformStamped.header.stamp = ros::Time::now();
		transformStamped.header.frame_id = "hub_link";

		float angle;
		bool color_is_red;

		switch(i)
		{
			case 0:
			{
				angle = 45;
				transformStamped.child_frame_id = "tarmac_blue_1_link";
				color_is_red = false;
			}
			break;
			case 1:
			{
				angle = 135;
				transformStamped.child_frame_id = "tarmac_red_2_link";
				color_is_red = true;
			}
			break;
			case 2:
			{
				angle = 225;
				transformStamped.child_frame_id = "tarmac_red_1_link";
				color_is_red = true;
			}
			break;
			default:
			case 3:
			{
				angle = 315;
				transformStamped.child_frame_id = "tarmac_blue_2_link";
				color_is_red = false;
			}
			break;
		}

		float offset_distance = 33.89 * INCHES_TO_METERS;

		transformStamped.transform.translation.x = cos(angle * DEGREES_TO_RADIANS) * offset_distance;
		transformStamped.transform.translation.y = -sin(angle * DEGREES_TO_RADIANS) * offset_distance;
		transformStamped.transform.translation.z = 0.0;

		tf2::Quaternion q;
		q.setRPY(0,0,-angle * DEGREES_TO_RADIANS);
		transformStamped.transform.rotation.x = q.x();
		transformStamped.transform.rotation.y = q.y();
		transformStamped.transform.rotation.z = q.z();
		transformStamped.transform.rotation.w = q.w();

		tfBroadcaster->sendTransform(transformStamped);

		publish_tarmac_lines(i, transformStamped.child_frame_id, color_is_red);

	}
}

void publish_hub_objects(void)
{
	publish_hub_link();
	publish_hub_full_height();
	publish_hub_cylinder();
	publish_tarmac_links();
}

void publisher_loop(void)
{
    while(ros::ok())
    {
		static ros::Rate rate_control(10);

		publish_red_link();
		publish_blue_link();
		publish_field_perimeter();
		publish_field_centerline();

		publish_hub_objects();

		rate_control.sleep();
	}
}

void create_tarmac_links()
{

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

	vis_pub = node->advertise<visualization_msgs::Marker> ("visualization_marker", 1);
    
	tfBroadcaster = new tf2_ros::TransformBroadcaster();

    std::thread publisher_thread(publisher_loop);

	ros::spin();
	return 0;
}