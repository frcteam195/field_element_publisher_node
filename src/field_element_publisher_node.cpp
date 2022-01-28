#include "ros/ros.h"
#include "std_msgs/String.h"

#include <thread>
#include <string>
#include <mutex>

#include <math.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>
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

	transformStamped.child_frame_id = "hub_unaligned";

	q.setRPY(0,0,0);
	transformStamped.transform.rotation.x = q.x();
	transformStamped.transform.rotation.y = q.y();
	transformStamped.transform.rotation.z = q.z();
	transformStamped.transform.rotation.w = q.w();

	tfBroadcaster->sendTransform(transformStamped);

	transformStamped.header.frame_id = "hub_link";
	transformStamped.child_frame_id = "red_hub_link";

	transformStamped.transform.translation.x = 0.0;
	transformStamped.transform.translation.y = 0.0;
	transformStamped.transform.translation.z = 0.0;

	q.setRPY(0,0,0);
	transformStamped.transform.rotation.x = q.x();
	transformStamped.transform.rotation.y = q.y();
	transformStamped.transform.rotation.z = q.z();
	transformStamped.transform.rotation.w = q.w();

	tfBroadcaster->sendTransform(transformStamped);

	transformStamped.header.frame_id = "hub_link";
	transformStamped.child_frame_id = "blue_hub_link";

	transformStamped.transform.translation.x = 0.0;
	transformStamped.transform.translation.y = 0.0;
	transformStamped.transform.translation.z = 0.0;

	q.setRPY(0,0,180.0 * DEGREES_TO_RADIANS);
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

void create_player_station()
{
	for (int i = 0; i < 2; i ++)
	{
		bool is_red = i == 0;

		visualization_msgs::Marker player_station;
		player_station.header.stamp = ros::Time::now();
		player_station.header.frame_id = is_red ? "red_link" : "blue_link";

		player_station.type = visualization_msgs::Marker::LINE_STRIP;
		player_station.action = visualization_msgs::Marker::ADD;
		player_station.id = 2 + i;
		player_station.ns = "FieldWall";
		player_station.color.r = is_red ? 1 : 0;
		player_station.color.g = 0;
		player_station.color.b = is_red ? 0 : 1;
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

	create_player_station();
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

	transformStamped.transform.translation.x = (69 * INCHES_TO_METERS) / 2.0;
	transformStamped.transform.translation.y = -((((27.0 * 12.0) - 252.0) / 2.0) + 252.0) * INCHES_TO_METERS;
	transformStamped.transform.translation.z = 0.0;

	tf2::Quaternion q;
	q.setRPY(0,0,43.781 * DEGREES_TO_RADIANS);
	transformStamped.transform.rotation.x = q.x();
	transformStamped.transform.rotation.y = q.y();
	transformStamped.transform.rotation.z = q.z();
	transformStamped.transform.rotation.w = q.w();

	tfBroadcaster->sendTransform(transformStamped);
}

void publish_terminal_cube(bool is_red)
{
	visualization_msgs::Marker terminal;

	if (is_red)
	{
		terminal.header.frame_id = "red_terminal_link";
	}
	else
	{
		terminal.header.frame_id = "blue_terminal_link";
	}

	terminal.header.stamp = ros::Time::now();
	terminal.ns = "terminals";
	terminal.id = is_red ? 0 : 1;
	terminal.type = visualization_msgs::Marker::CUBE;
	terminal.action = visualization_msgs::Marker::ADD;

	terminal.pose.position.x = -(39.0 / 2.0) * INCHES_TO_METERS;
	terminal.pose.position.y = 0;
	terminal.pose.position.z = (42.42 / 2.0) * INCHES_TO_METERS;

	tf2::Quaternion q;
	q.setRPY(0,0,0);

	terminal.pose.orientation.x = q.x();
	terminal.pose.orientation.y = q.y();
	terminal.pose.orientation.z = q.z();
	terminal.pose.orientation.w = q.w();

	terminal.scale.x = 39.0 * INCHES_TO_METERS;
	terminal.scale.y = 97.13 * INCHES_TO_METERS;
	terminal.scale.z = 42.42 * INCHES_TO_METERS;

	terminal.color.r = 0.7;
	terminal.color.g = 0.7;
	terminal.color.b = 0.7;
	terminal.color.a = 1.0;

	vis_pub.publish(terminal);
}

void publish_red_hangar_link (void)
{
	geometry_msgs::TransformStamped transformStamped;

	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = "red_link";
	transformStamped.child_frame_id = "red_hangar_link";

	transformStamped.transform.translation.x = (10.0625 / 2.0) * FEET_TO_METERS; 
	transformStamped.transform.translation.y = (-10.020833 / 2.0) * FEET_TO_METERS;
	transformStamped.transform.translation.z = 0.0;

	tf2::Quaternion q;
	q.setRPY(0,0,0 * DEGREES_TO_RADIANS);
	transformStamped.transform.rotation.x = q.x();
	transformStamped.transform.rotation.y = q.y();
	transformStamped.transform.rotation.z = q.z();
	transformStamped.transform.rotation.w = q.w();

	tfBroadcaster->sendTransform(transformStamped);
}

void publish_one_pillar_red(int a, int b, int p)
{
	visualization_msgs::Marker hangar;
	hangar.header.frame_id = "red_hangar_link";
	hangar.header.stamp = ros::Time::now();
	hangar.ns = "Hangar";;
	hangar.id = p;
	hangar.type = visualization_msgs::Marker::CUBE;
	hangar.action = visualization_msgs::Marker::ADD;

	hangar.pose.position.x = a * FEET_TO_METERS;
	hangar.pose.position.y = b * FEET_TO_METERS;
	hangar.pose.position.z = 6.1667/2 * FEET_TO_METERS;

	tf2::Quaternion q;
	q.setRPY(0,0,0);

	hangar.pose.orientation.x = q.x();
	hangar.pose.orientation.y = q.y();
	hangar.pose.orientation.z = q.z();
	hangar.pose.orientation.w = q.w();

	hangar.scale.x = 1 * FEET_TO_METERS;
	hangar.scale.y = 1 * FEET_TO_METERS;
	hangar.scale.z = 6.1667 * FEET_TO_METERS;

	hangar.color.r = 0.7;
	hangar.color.g = 0.7;
	hangar.color.b = 0.7;
	hangar.color.a = 1.0;

	vis_pub.publish(hangar);
}

void publish_hangar_pillars_red(void)
{

	publish_one_pillar_red(-4.5625, -4.09375, 0);
	publish_one_pillar_red(-4.5625, 4.09375, 1);
	publish_one_pillar_red(4.5625, -4.09375, 2);
	publish_one_pillar_red(4.5625, 4.09375, 3);

	visualization_msgs::Marker hangar;
	hangar.header.frame_id = "red_hangar_link";
	hangar.header.stamp = ros::Time::now();
	hangar.ns = "Hangar";;
	hangar.id = 4;
	hangar.type = visualization_msgs::Marker::CUBE;
	hangar.action = visualization_msgs::Marker::ADD;

	hangar.pose.position.x = 0;
	hangar.pose.position.y = 4.005215* FEET_TO_METERS;
	hangar.pose.position.z = 1.725;

	tf2::Quaternion q;
	q.setRPY(0,0,0);

	hangar.pose.orientation.x = q.x();
	hangar.pose.orientation.y = q.y();
	hangar.pose.orientation.z = q.z();
	hangar.pose.orientation.w = q.w();

	hangar.scale.x = 9.125 * FEET_TO_METERS;
	hangar.scale.y = 1 * FEET_TO_METERS;
	hangar.scale.z = 1 * FEET_TO_METERS;

	hangar.color.r = 0.7;
	hangar.color.g = 0.7;
	hangar.color.b = 0.7;
	hangar.color.a = 1.0;

	vis_pub.publish(hangar);
}

void publish_hangar_connector_red(void)
{
	visualization_msgs::Marker hangar;
	hangar.header.frame_id = "red_hangar_link";
	hangar.header.stamp = ros::Time::now();
	hangar.ns = "hangar";;
	hangar.id = 5;
	hangar.type = visualization_msgs::Marker::CUBE;
	hangar.action = visualization_msgs::Marker::ADD;

	hangar.pose.position.x = 0;
	hangar.pose.position.y = -4.005215* FEET_TO_METERS;
	hangar.pose.position.z = 1.725;

	tf2::Quaternion q;
	q.setRPY(0,0,0);

	hangar.pose.orientation.x = q.x();
	hangar.pose.orientation.y = q.y();
	hangar.pose.orientation.z = q.z();
	hangar.pose.orientation.w = q.w();

	hangar.scale.x = 9.125 * FEET_TO_METERS;
	hangar.scale.y = 1 * FEET_TO_METERS;
	hangar.scale.z = 1 * FEET_TO_METERS;

	hangar.color.r = 0.7;
	hangar.color.g = 0.7;
	hangar.color.b = 0.7;
	hangar.color.a = 1.0;

	vis_pub.publish(hangar);
}

void publish_one_rung_red(int x, int z, int i)
{
	visualization_msgs::Marker rung;
	rung.header.frame_id = "red_hangar_link";
	rung.header.stamp = ros::Time::now();
	rung.ns = "rung";
	rung.id = i;
	rung.type = visualization_msgs::Marker::CYLINDER;
	rung.action = visualization_msgs::Marker::ADD;

	rung.pose.position.x = x * INCHES_TO_METERS;
	rung.pose.position.y = 0;
	rung.pose.position.z = z * INCHES_TO_METERS;

	tf2::Quaternion q;
	q.setRPY(90 * DEGREES_TO_RADIANS, 0, 0);

	rung.pose.orientation.x = q.x();
	rung.pose.orientation.y = q.y();
	rung.pose.orientation.z = q.z();
	rung.pose.orientation.w = q.w();

	rung.scale.x = 1.66 * INCHES_TO_METERS;
	rung.scale.y = 1.66 * INCHES_TO_METERS;
	rung.scale.z = 7 * FEET_TO_METERS;

	rung.color.r = 0.7;
	rung.color.g = 0.7;
	rung.color.b = 0.7;
	rung.color.a = 1.0;

	vis_pub.publish(rung);
}

void publish_rungs_red(void) {
	publish_one_rung_red(60.41, 48, 1);
	publish_one_rung_red(20.75, 60, 2);
	publish_one_rung_red(-3.27, 75.33, 3);
	publish_one_rung_red(-27.25, 90.67, 4);
}




void publish_blue_hangar_link (void)
{
	geometry_msgs::TransformStamped transformStamped;

	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = "blue_link";
	transformStamped.child_frame_id = "blue_hangar_link";

	transformStamped.transform.translation.x = (10.0625 / 2.0) * FEET_TO_METERS;
	transformStamped.transform.translation.y = (-10.020833 / 2.0) * FEET_TO_METERS;
	transformStamped.transform.translation.z = 0.0;

	tf2::Quaternion q;
	q.setRPY(0,0,0 * DEGREES_TO_RADIANS);
	transformStamped.transform.rotation.x = q.x();
	transformStamped.transform.rotation.y = q.y();
	transformStamped.transform.rotation.z = q.z();
	transformStamped.transform.rotation.w = q.w();

	tfBroadcaster->sendTransform(transformStamped);
}

void publish_one_pillar_blue(int a, int b, int p)
{
	visualization_msgs::Marker hangar;
	hangar.header.frame_id = "blue_hangar_link";
	hangar.header.stamp = ros::Time::now();
	hangar.ns = "Hangar";;
	hangar.id = p;
	hangar.type = visualization_msgs::Marker::CUBE;
	hangar.action = visualization_msgs::Marker::ADD;

	hangar.pose.position.x = a * FEET_TO_METERS;
	hangar.pose.position.y = b * FEET_TO_METERS;
	hangar.pose.position.z = 6.1667/2 * FEET_TO_METERS;

	tf2::Quaternion q;
	q.setRPY(0,0,0);

	hangar.pose.orientation.x = q.x();
	hangar.pose.orientation.y = q.y();
	hangar.pose.orientation.z = q.z();
	hangar.pose.orientation.w = q.w();

	hangar.scale.x = 1 * FEET_TO_METERS;
	hangar.scale.y = 1 * FEET_TO_METERS;
	hangar.scale.z = 6.1667 * FEET_TO_METERS;

	hangar.color.r = 0.7;
	hangar.color.g = 0.7;
	hangar.color.b = 0.7;
	hangar.color.a = 1.0;

	vis_pub.publish(hangar);
}

void publish_hangar_pillars_blue(void)
{

	publish_one_pillar_blue(-4.5625, -4.09375, 10);
	publish_one_pillar_blue(-4.5625, 4.09375, 6);
	publish_one_pillar_blue(4.5625, -4.09375, 7);
	publish_one_pillar_blue(4.5625, 4.09375, 9);

	visualization_msgs::Marker hangar;
	hangar.header.frame_id = "blue_hangar_link";
	hangar.header.stamp = ros::Time::now();
	hangar.ns = "Hangar";;
	hangar.id = 8;
	hangar.type = visualization_msgs::Marker::CUBE;
	hangar.action = visualization_msgs::Marker::ADD;

	hangar.pose.position.x = 0;
	hangar.pose.position.y = 4.005215* FEET_TO_METERS;
	hangar.pose.position.z = 1.725;

	tf2::Quaternion q;
	q.setRPY(0,0,0);

	hangar.pose.orientation.x = q.x();
	hangar.pose.orientation.y = q.y();
	hangar.pose.orientation.z = q.z();
	hangar.pose.orientation.w = q.w();

	hangar.scale.x = 9.125 * FEET_TO_METERS;
	hangar.scale.y = 1 * FEET_TO_METERS;
	hangar.scale.z = 1 * FEET_TO_METERS;

	hangar.color.r = 0.7;
	hangar.color.g = 0.7;
	hangar.color.b = 0.7;
	hangar.color.a = 1.0;

	vis_pub.publish(hangar);
}

void publish_hangar_connector_blue(void)
{
	visualization_msgs::Marker hangar;
	hangar.header.frame_id = "blue_hangar_link";
	hangar.header.stamp = ros::Time::now();
	hangar.ns = "hangar";;
	hangar.id = 7;
	hangar.type = visualization_msgs::Marker::CUBE;
	hangar.action = visualization_msgs::Marker::ADD;

	hangar.pose.position.x = 0;
	hangar.pose.position.y = -4.005215* FEET_TO_METERS;
	hangar.pose.position.z = 1.725;

	tf2::Quaternion q;
	q.setRPY(0,0,0);

	hangar.pose.orientation.x = q.x();
	hangar.pose.orientation.y = q.y();
	hangar.pose.orientation.z = q.z();
	hangar.pose.orientation.w = q.w();

	hangar.scale.x = 9.125 * FEET_TO_METERS;
	hangar.scale.y = 1 * FEET_TO_METERS;
	hangar.scale.z = 1 * FEET_TO_METERS;

	hangar.color.r = 0.7;
	hangar.color.g = 0.7;
	hangar.color.b = 0.7;
	hangar.color.a = 1.0;

	vis_pub.publish(hangar);
}

void publish_one_rung_blue(int x, int z, int i)
{
	visualization_msgs::Marker rung;
	rung.header.frame_id = "blue_hangar_link";
	rung.header.stamp = ros::Time::now();
	rung.ns = "rung";
	rung.id = i;
	rung.type = visualization_msgs::Marker::CYLINDER;
	rung.action = visualization_msgs::Marker::ADD;

	rung.pose.position.x = x * INCHES_TO_METERS;
	rung.pose.position.y = 0;
	rung.pose.position.z = z * INCHES_TO_METERS;

	tf2::Quaternion q;
	q.setRPY(90 * DEGREES_TO_RADIANS, 0, 0);

	rung.pose.orientation.x = q.x();
	rung.pose.orientation.y = q.y();
	rung.pose.orientation.z = q.z();
	rung.pose.orientation.w = q.w();

	rung.scale.x = 1.66 * INCHES_TO_METERS;
	rung.scale.y = 1.66 * INCHES_TO_METERS;
	rung.scale.z = 7 * FEET_TO_METERS;

	rung.color.r = 0.7;
	rung.color.g = 0.7;
	rung.color.b = 0.7;
	rung.color.a = 1.0;

	vis_pub.publish(rung);
}

void publish_robot_box(){
	visualization_msgs::Marker rung;
	rung.header.frame_id = "base_link";
	rung.header.stamp = ros::Time::now();
	rung.ns = "robot";
	rung.id = 0;
	rung.type = visualization_msgs::Marker::CUBE;
	rung.action = visualization_msgs::Marker::ADD;

	rung.pose.position.x = 0;
	rung.pose.position.y = 0;
	rung.pose.position.z = 2 * INCHES_TO_METERS;

	tf2::Quaternion q;
	q.setRPY(0 * DEGREES_TO_RADIANS, 0, 0);

	rung.pose.orientation.x = q.x();
	rung.pose.orientation.y = q.y();
	rung.pose.orientation.z = q.z();
	rung.pose.orientation.w = q.w();

	rung.scale.x = 36 * INCHES_TO_METERS;
	rung.scale.y = 32 * INCHES_TO_METERS;
	rung.scale.z = 4 * INCHES_TO_METERS;

	rung.color.r = 0.7;
	rung.color.g = 0.7;
	rung.color.b = 0.7;
	rung.color.a = 1.0;

	vis_pub.publish(rung);
}

void publish_robot_topbox(){
	visualization_msgs::Marker rung;
	rung.header.frame_id = "base_link";
	rung.header.stamp = ros::Time::now();
	rung.ns = "robot";
	rung.id = 1;
	rung.type = visualization_msgs::Marker::CUBE;
	rung.action = visualization_msgs::Marker::ADD;

	rung.pose.position.x = 0;
	rung.pose.position.y = 0;
	rung.pose.position.z = 7* INCHES_TO_METERS;

	tf2::Quaternion q;
	q.setRPY(0 * DEGREES_TO_RADIANS, 0, 0);

	rung.pose.orientation.x = q.x();
	rung.pose.orientation.y = q.y();
	rung.pose.orientation.z = q.z();
	rung.pose.orientation.w = q.w();

	rung.scale.x = 31 * INCHES_TO_METERS;
	rung.scale.y = 24 * INCHES_TO_METERS;
	rung.scale.z = 14 * INCHES_TO_METERS;

	rung.color.r = 0.7;
	rung.color.g = 0.7;
	rung.color.b = 0.7;
	rung.color.a = 1.0;

	vis_pub.publish(rung);
}

void publish_robot_turret(){
	visualization_msgs::Marker rung;
	rung.header.frame_id = "base_link";
	rung.header.stamp = ros::Time::now();
	rung.ns = "robot";
	rung.id = 2;
	rung.type = visualization_msgs::Marker::CUBE;
	rung.action = visualization_msgs::Marker::ADD;

	rung.pose.position.x = 0;
	rung.pose.position.y = 0;
	rung.pose.position.z = 12.5* INCHES_TO_METERS;

	tf2::Quaternion q;
	q.setRPY(0 * DEGREES_TO_RADIANS, 0, 0);

	rung.pose.orientation.x = q.x();
	rung.pose.orientation.y = q.y();
	rung.pose.orientation.z = q.z();
	rung.pose.orientation.w = q.w();

	rung.scale.x = 18* INCHES_TO_METERS;
	rung.scale.y = 13 * INCHES_TO_METERS;
	rung.scale.z = 25* INCHES_TO_METERS;

	rung.color.r = 0.7;
	rung.color.g = 0.7;
	rung.color.b = 0.7;
	rung.color.a = 1.0;

	vis_pub.publish(rung);
}






void publish_rungs_blue(void) {
	publish_one_rung_blue(60.41, 48, 5);
	publish_one_rung_blue(20.75, 60, 6);
	publish_one_rung_blue(-3.27, 75.33, 7);
	publish_one_rung_blue(-27.25, 90.67, 8);
}


void publish_terminal_ball_link()
{
	for (int i = 0; i < 2; i ++)
	{
		bool is_red = i == 0;

		geometry_msgs::TransformStamped transformStamped;

		transformStamped.header.stamp = ros::Time::now();
		transformStamped.header.frame_id = is_red ? "red_terminal_link" : "blue_terminal_link";
		transformStamped.child_frame_id = is_red ? "red_ball_7" : "blue_ball_7";

		transformStamped.transform.translation.x = 10.43 * INCHES_TO_METERS;
		transformStamped.transform.translation.y = 0;
		transformStamped.transform.translation.z = 0.0;

		tf2::Quaternion q;
		q.setRPY(0,0,0 * DEGREES_TO_RADIANS);
		transformStamped.transform.rotation.x = q.x();
		transformStamped.transform.rotation.y = q.y();
		transformStamped.transform.rotation.z = q.z();
		transformStamped.transform.rotation.w = q.w();

		tfBroadcaster->sendTransform(transformStamped);
	}
}

void publish_auto_1_link()
{
	for (int i = 0; i < 2; i ++)
	{
		bool is_red = i == 0;

		geometry_msgs::TransformStamped transformStamped;

		transformStamped.header.stamp = ros::Time::now();
		transformStamped.header.frame_id = is_red ? "red_link" : "blue_link";
		transformStamped.child_frame_id = is_red ? "auto_1_red_link" : "auto_1_blue_link";

		transformStamped.transform.translation.x = 260 * INCHES_TO_METERS;
		transformStamped.transform.translation.y = -97.5 * INCHES_TO_METERS;
		transformStamped.transform.translation.z = 0.0;

		tf2::Quaternion q;
		q.setRPY(0,0,(90 + 33 + 20) * DEGREES_TO_RADIANS);
		transformStamped.transform.rotation.x = q.x();
		transformStamped.transform.rotation.y = q.y();
		transformStamped.transform.rotation.z = q.z();
		transformStamped.transform.rotation.w = q.w();

		tfBroadcaster->sendTransform(transformStamped);
	}
}

void publish_cargo_line_ball_links()
{
	for (int j = 0; j < 2; j ++)
	{
		bool is_red = j == 0;

		for(int i = 0; i < 3; i++)
		{
			geometry_msgs::TransformStamped transformStamped;

			std::stringstream ss;
			ss << (is_red ? "red_" : "blue_") << "ball_" << (8 + i);

			transformStamped.header.stamp = ros::Time::now();
			transformStamped.header.frame_id = is_red ? "red_link" : "blue_link";
			transformStamped.child_frame_id = ss.str();

			transformStamped.transform.translation.x = 6.0 * INCHES_TO_METERS;
			transformStamped.transform.translation.y = (-204.0 * INCHES_TO_METERS) - (6.0 * INCHES_TO_METERS) - ((i * 12.0) * INCHES_TO_METERS);
			transformStamped.transform.translation.z = 0.0;

			tf2::Quaternion q;
			q.setRPY(0,0,0 * DEGREES_TO_RADIANS);
			transformStamped.transform.rotation.x = q.x();
			transformStamped.transform.rotation.y = q.y();
			transformStamped.transform.rotation.z = q.z();
			transformStamped.transform.rotation.w = q.w();

			tfBroadcaster->sendTransform(transformStamped);
		}
	}
}

void publish_hub_ball_links(bool is_red)
{
	geometry_msgs::TransformStamped transformStamped;
	std::string link_prefix;

	transformStamped.header.stamp = ros::Time::now();

	switch (is_red)
	{
		case true:
		{
			transformStamped.header.frame_id = "red_hub_link";
			link_prefix = "red_";
		}
		break;

		case false:
		{
			transformStamped.header.frame_id = "blue_hub_link";
			link_prefix = "blue_";
		}
		break;
	}

	float angle;
	float angle_1 = 11.25;
	float angle_2 = 22.5;

	for (int i = 0; i < 6; i ++)
	{
		switch(i)
		{
			case 0:
			{
				transformStamped.child_frame_id = link_prefix + "ball_1";
				angle = (-90 - angle_1 - angle_2) * DEGREES_TO_RADIANS;
			}
			break;
			case 1:
			{
				transformStamped.child_frame_id = link_prefix + "ball_2";
				angle = (-180 - angle_1) * DEGREES_TO_RADIANS;
			}
			break;
			case 2:
			{
				transformStamped.child_frame_id = link_prefix + "ball_3";
				angle = (-270 + angle_1 + angle_2) * DEGREES_TO_RADIANS;
			}
			break;
			case 3:
			{
				transformStamped.child_frame_id = link_prefix + "ball_4";
				angle = (-270 - angle_1) * DEGREES_TO_RADIANS;
			}
			break;
			case 4:
			{
				transformStamped.child_frame_id = link_prefix + "ball_5";
				angle = angle_1 * DEGREES_TO_RADIANS;
			}
			break;
			case 5:
			default:
			{
				transformStamped.child_frame_id = link_prefix + "ball_6";
				angle = (-90 + angle_1) * DEGREES_TO_RADIANS;
			}
			break;
		}

		geometry_msgs::Point base;
		base.x = 0;
		base.y = 0;
		base.z = 0;

		geometry_msgs::Point result = compute_offset(base, angle, 153.0 * INCHES_TO_METERS);

		transformStamped.transform.translation.x = result.x;
		transformStamped.transform.translation.y = result.y;
		transformStamped.transform.translation.z = 0.0;

		tf2::Quaternion q;
		q.setRPY(0,0,-angle);
		transformStamped.transform.rotation.x = q.x();
		transformStamped.transform.rotation.y = q.y();
		transformStamped.transform.rotation.z = q.z();
		transformStamped.transform.rotation.w = q.w();

		tfBroadcaster->sendTransform(transformStamped);
	}
}

void render_balls()
{
	for (int j = 0; j < 2; j++)
	{
		std::string prefix = j == 0 ? "red_" : "blue_";

		std_msgs::ColorRGBA color;
		color.r = j == 0 ? 1 : 0;
		color.g = 0;
		color.b = j == 0 ? 0 : 1;
		color.a = 1.0;

		for (int i = 0; i < 10; i++)
		{
			std::stringstream ss;
			ss << prefix << "ball_" << i + 1;

			visualization_msgs::Marker ball;
			ball.header.frame_id = ss.str();

			ball.header.stamp = ros::Time::now();
			ball.ns = "balls";;
			ball.id = (j * 10) + i;
			ball.type = visualization_msgs::Marker::SPHERE;
			ball.action = visualization_msgs::Marker::ADD;

			ball.pose.position.x = 0;
			ball.pose.position.y = 0;
			ball.pose.position.z = .2413 / 2.0;

			tf2::Quaternion q;
			q.setRPY(0,0,0);

			ball.pose.orientation.x = q.x();
			ball.pose.orientation.y = q.y();
			ball.pose.orientation.z = q.z();
			ball.pose.orientation.w = q.w();

			ball.scale.x = .2413;
			ball.scale.y = .2413;
			ball.scale.z = .2413;

			ball.color = color;

			vis_pub.publish(ball);
		}
	}
}

void publish_cargo_line()
{
	for (int i = 0; i < 2; i++)
	{
		bool is_red = i == 0;

		visualization_msgs::Marker center_line;
		center_line.header.stamp = ros::Time::now();
		center_line.header.frame_id = is_red ? "red_link" : "blue_link";

		center_line.type = visualization_msgs::Marker::LINE_STRIP;
		center_line.action = visualization_msgs::Marker::ADD;
		center_line.id = 7 + i;
		center_line.ns = "FieldWall";
		center_line.color.r = 0;
		center_line.color.g = 0;
		center_line.color.b = 0;
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
		field_point.x = 12 * INCHES_TO_METERS;
		field_point.y = -204 * INCHES_TO_METERS;
		field_point.z = 0;
		center_line.points.push_back(field_point);

		field_point.x = 12 * INCHES_TO_METERS;
		field_point.y = (-204 - 36) * INCHES_TO_METERS;
		field_point.z = 0;
		center_line.points.push_back(field_point);

		vis_pub.publish(center_line);
	}
}

void publish_hangar_objects(void)
{
	publish_red_hangar_link();
	publish_hangar_pillars_red();
	publish_hangar_connector_red();
	publish_rungs_red();

	publish_blue_hangar_link();
	publish_hangar_pillars_blue();
	publish_hangar_connector_blue();
	publish_rungs_blue();
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
		publish_terminal_link(true);
		publish_terminal_link(false);
		publish_terminal_cube(true);
		publish_terminal_cube(false);
		publish_hub_ball_links(true);
		publish_hub_ball_links(false);
		publish_terminal_ball_link();
		publish_cargo_line();
		publish_cargo_line_ball_links();
		publish_auto_1_link();
		render_balls();
		publish_robot_box();
		publish_robot_topbox();

		publish_robot_turret();
		publish_hangar_objects();

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