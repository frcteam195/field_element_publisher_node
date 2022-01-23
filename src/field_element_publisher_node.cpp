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

geometry_msgs::Point compute_offset(geometry_msgs::Point base, float angle, float distance)
{
	geometry_msgs::Point result = base;

	result.x += cos(angle) * distance;
	result.y += -sin(angle) * distance;

	return result;
}

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

void create_player_station(int base_id, std::string base_link)
{
	visualization_msgs::Marker player_station;
	player_station.header.stamp = ros::Time::now();
	player_station.header.frame_id = base_link;

	player_station.type = visualization_msgs::Marker::LINE_STRIP;
	player_station.action = visualization_msgs::Marker::ADD;
	player_station.id = base_id;
	player_station.ns = "FieldWall";
	player_station.color.r = 1;
	player_station.color.g = 1;
	player_station.color.b = 1;
	player_station.color.a = 1;
	player_station.scale.x = .03;

	tf2::Quaternion q;
	q.setRPY(0,0,0);

	player_station.pose.orientation.x = q.x();
	player_station.pose.orientation.y = q.y();
	player_station.pose.orientation.z = q.z();
	player_station.pose.orientation.w = q.w();

	player_station.pose.position.x = 0;
	player_station.pose.position.y = 0;
	player_station.pose.position.z = 0;

	geometry_msgs::Point field_point;
	field_point.x = 0;
	field_point.y = 0;
	field_point.z = 0;
	player_station.points.push_back(field_point);

	field_point.x = 0;
	field_point.y = 0;
	field_point.z = 77.63 * INCHES_TO_METERS;
	player_station.points.push_back(field_point);

	field_point.x = 0;
	field_point.y = -249.64 * INCHES_TO_METERS;
	field_point.z = 77.63 * INCHES_TO_METERS;
	player_station.points.push_back(field_point);

	field_point.x = 0;
	field_point.y = -249.64 * INCHES_TO_METERS;
	field_point.z = 0;
	player_station.points.push_back(field_point);

	vis_pub.publish(player_station);
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

	visualization_msgs::Marker field_perimeter2;
	field_perimeter2.header.stamp = ros::Time::now();
	field_perimeter2.header.frame_id = "red_link";

	field_perimeter2.type = visualization_msgs::Marker::LINE_STRIP;
	field_perimeter2.action = visualization_msgs::Marker::ADD;
	field_perimeter2.id = 1;
	field_perimeter2.ns = "FieldWall";
	field_perimeter2.color.r = 1;
	field_perimeter2.color.g = 1;
	field_perimeter2.color.b = 1;
	field_perimeter2.color.a = 1;
	field_perimeter2.scale.x = .03;

	q.setRPY(0,0,0);

	field_perimeter2.pose.orientation.x = q.x();
	field_perimeter2.pose.orientation.y = q.y();
	field_perimeter2.pose.orientation.z = q.z();
	field_perimeter2.pose.orientation.w = q.w();

	field_perimeter2.pose.position.x = 0;
	field_perimeter2.pose.position.y = 0;
	field_perimeter2.pose.position.z = 0;

	field_point.x = 0;
	field_point.y = 0;
	field_point.z = 20 * INCHES_TO_METERS;
	field_perimeter2.points.push_back(field_point);

	field_point.x = 0;
	field_point.y = -252 * INCHES_TO_METERS;
	field_point.z = 20 * INCHES_TO_METERS;
	field_perimeter2.points.push_back(field_point);

	field_point.x = 69 * INCHES_TO_METERS;
	field_point.y = -27 * FEET_TO_METERS;
	field_point.z = 20 * INCHES_TO_METERS;
	field_perimeter2.points.push_back(field_point);

	field_point.x = 54 * FEET_TO_METERS;
	field_point.y = -27 * FEET_TO_METERS;
	field_point.z = 20 * INCHES_TO_METERS;
	field_perimeter2.points.push_back(field_point);

	field_point.x = 54 * FEET_TO_METERS;
	field_point.y = (-27 * FEET_TO_METERS) + (252 * INCHES_TO_METERS);
	field_point.z = 20 * INCHES_TO_METERS;
	field_perimeter2.points.push_back(field_point);

	field_point.x = (54 * FEET_TO_METERS) - (69 * INCHES_TO_METERS);
	field_point.y = 0;
	field_point.z = 20 * INCHES_TO_METERS;
	field_perimeter2.points.push_back(field_point);

	field_point.x = 0;
	field_point.y = 0;
	field_point.z = 20 * INCHES_TO_METERS;
	field_perimeter2.points.push_back(field_point);

	vis_pub.publish(field_perimeter2);

	create_player_station(2, "red_link");
	create_player_station(3, "blue_link");
}

void publish_field_centerline()
{

	visualization_msgs::Marker center_line;
	center_line.header.stamp = ros::Time::now();
	center_line.header.frame_id = "hub_link";

	center_line.type = visualization_msgs::Marker::LINE_STRIP;
	center_line.action = visualization_msgs::Marker::ADD;
	center_line.id = 15;
	center_line.ns = "Hub";
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
	hub.ns = "Hub";
	hub.id = 5;
	hub.type = visualization_msgs::Marker::CUBE;
	hub.action = visualization_msgs::Marker::ADD;

	hub.pose.position.x = 0;
	hub.pose.position.y = 0;
	hub.pose.position.z = (22.5 * INCHES_TO_METERS) / 2;

	tf2::Quaternion q;
	q.setRPY(0,0,45 * DEGREES_TO_RADIANS);

	hub.pose.orientation.x = q.x();
	hub.pose.orientation.y = q.y();
	hub.pose.orientation.z = q.z();
	hub.pose.orientation.w = q.w();

	hub.scale.x = 67.77 * INCHES_TO_METERS;
	hub.scale.y = 67.77 * INCHES_TO_METERS;
	hub.scale.z = 22.5 * INCHES_TO_METERS;

	hub.color.r = 0.7;
	hub.color.g = 0.7;
	hub.color.b = 0.7;
	hub.color.a = 1.0;

	vis_pub.publish(hub);
}

void publish_hub_lower_cylinder(void)
{
	visualization_msgs::Marker hub;
	hub.header.frame_id = "hub_link";
	hub.header.stamp = ros::Time::now();
	hub.ns = "Hub";
	hub.id = 6;
	hub.type = visualization_msgs::Marker::CYLINDER;
	hub.action = visualization_msgs::Marker::ADD;

	hub.pose.position.x = 0;
	hub.pose.position.y = 0;
	hub.pose.position.z = 1.04 / 2.0;

	tf2::Quaternion q;
	q.setRPY(0,0,45 * DEGREES_TO_RADIANS);

	hub.pose.orientation.x = q.x();
	hub.pose.orientation.y = q.y();
	hub.pose.orientation.z = q.z();
	hub.pose.orientation.w = q.w();

	hub.scale.x = 31.88 * 2.0 * INCHES_TO_METERS;
	hub.scale.y = 31.88 * 2.0 * INCHES_TO_METERS;
	hub.scale.z = 1.04;

	hub.color.r = 0.7;
	hub.color.g = 0.7;
	hub.color.b = 0.7;
	hub.color.a = 1.0;

	vis_pub.publish(hub);
}

void publish_hub_upper_return(int base_id, float angle)
{
	visualization_msgs::Marker hub;
	hub.header.frame_id = "hub_link";
	hub.header.stamp = ros::Time::now();
	hub.ns = "Hub";
	hub.id = base_id + 7;
	hub.type = visualization_msgs::Marker::CUBE;
	hub.action = visualization_msgs::Marker::ADD;

	geometry_msgs::Point base;
	base.x = 0;
	base.y = 0;
	base.z = 1.71;

	geometry_msgs::Point center_point = compute_offset(base, angle * DEGREES_TO_RADIANS, (67.68 / 2.0) * INCHES_TO_METERS);

	hub.pose.position = center_point;

	tf2::Quaternion q;
	q.setRPY(0,0,-angle * DEGREES_TO_RADIANS);

	hub.pose.orientation.x = q.x();
	hub.pose.orientation.y = q.y();
	hub.pose.orientation.z = q.z();
	hub.pose.orientation.w = q.w();

	hub.scale.x = 67.68 * INCHES_TO_METERS;
	hub.scale.y = 30.98 * INCHES_TO_METERS;
	hub.scale.z = 0.1;

	hub.color.r = 0.7;
	hub.color.g = 0.7;
	hub.color.b = 0.7;
	hub.color.a = 1.0;

	vis_pub.publish(hub);
}

void publish_hub_upper_returns(void)
{
	for(int i = 0; i < 4; i++)
	{
		float angle;
		switch(i)
		{
			case 0:
			{
				angle = 0;
			}
			break;
			case 1:
			{
				angle = 90;
			}
			break;
			case 2:
			{
				angle = 180;
			}
			break;
			case 3:
			{
				angle = 270;
			}
			break;
		}

		publish_hub_upper_return(i, angle);
	}
}

void publish_hub_lower_return(int base_id, float angle)
{
	visualization_msgs::Marker hub;
	hub.header.frame_id = "hub_link";
	hub.header.stamp = ros::Time::now();
	hub.ns = "Hub";
	hub.id = base_id + 11;
	hub.type = visualization_msgs::Marker::CUBE;
	hub.action = visualization_msgs::Marker::ADD;

	geometry_msgs::Point base;
	base.x = 0;
	base.y = 0;
	base.z = 0.57 / 2.0;

	geometry_msgs::Point center_point = compute_offset(base, angle * DEGREES_TO_RADIANS, (53.5 / 2.0) * INCHES_TO_METERS);

	hub.pose.position = center_point;

	tf2::Quaternion q;
	q.setRPY(0,0,-angle * DEGREES_TO_RADIANS);

	hub.pose.orientation.x = q.x();
	hub.pose.orientation.y = q.y();
	hub.pose.orientation.z = q.z();
	hub.pose.orientation.w = q.w();

	hub.scale.x = 53.5 * INCHES_TO_METERS;
	hub.scale.y = 14 * INCHES_TO_METERS;
	hub.scale.z = 0.57;

	hub.color.r = 0.7;
	hub.color.g = 0.7;
	hub.color.b = 0.7;
	hub.color.a = 1.0;

	vis_pub.publish(hub);
}

void publish_hub_lower_returns(void)
{
	for(int i = 0; i < 4; i++)
	{
		float angle;
		switch(i)
		{
			case 0:
			{
				angle = 0;
			}
			break;
			case 1:
			{
				angle = 90;
			}
			break;
			case 2:
			{
				angle = 180;
			}
			break;
			case 3:
			{
				angle = 270;
			}
			break;
		}

		publish_hub_lower_return(i, angle);
	}
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

void publish_terminal_link(bool is_red)
{
	geometry_msgs::TransformStamped transformStamped;

	std::string base_link;

	if (is_red)
	{
		transformStamped.header.frame_id = "red_link";
		transformStamped.child_frame_id = "red_terminal_link";
	}
	else
	{
		transformStamped.header.frame_id = "blue_link";
		transformStamped.child_frame_id = "blue_terminal_link";
	}

	transformStamped.header.stamp = ros::Time::now();

	transformStamped.transform.translation.x = 69 * INCHES_TO_METERS;
	transformStamped.transform.translation.y = (((27 * 12 - 252) / 2) + 252) * INCHES_TO_METERS;
	transformStamped.transform.translation.z = 0.0;

	tf2::Quaternion q;
	q.setRPY(0,0,0);
	transformStamped.transform.rotation.x = q.x();
	transformStamped.transform.rotation.y = q.y();
	transformStamped.transform.rotation.z = q.z();
	transformStamped.transform.rotation.w = q.w();

	tfBroadcaster->sendTransform(transformStamped);
}

void publish_hub_objects(void)
{
	publish_hub_link();
	publish_hub_full_height();
	publish_hub_cylinder();
	publish_hub_lower_cylinder();
	publish_hub_base();
	publish_hub_upper_returns();
	publish_hub_lower_returns();
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

	vis_pub = node->advertise<visualization_msgs::Marker> ("visualization_marker", 100);

	tfBroadcaster = new tf2_ros::TransformBroadcaster();

    std::thread publisher_thread(publisher_loop);

	ros::spin();
	return 0;
}