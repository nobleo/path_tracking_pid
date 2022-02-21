#include "path_tracking_pid/visualization.hpp"

#include <string>

#include "ros/node_handle.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "visualization_msgs/Marker.h"

namespace path_tracking_pid
{
Visualization::Visualization(ros::NodeHandle nh)
{
  marker_pub_ = nh.advertise<visualization_msgs::Marker>("visualization_marker", 4);
}

void Visualization::publishControlPoint(
  const std_msgs::Header & header, const tf2::Transform & pose)
{
  std_msgs::ColorRGBA color;
  color.a = 1.0;
  color.g = 1.0;
  // id has to be unique, so using a compile-time counter :)
  publishSphere(header, "control point", __COUNTER__, pose, color);
}

void Visualization::publishAxlePoint(const std_msgs::Header & header, const tf2::Transform & pose)
{
  std_msgs::ColorRGBA color;
  color.a = 1.0;
  color.b = 1.0;
  publishSphere(header, "axle point", __COUNTER__, pose, color);
}

void Visualization::publishGoalPoint(const std_msgs::Header & header, const tf2::Transform & pose)
{
  std_msgs::ColorRGBA color;
  color.a = 1.0;
  color.r = 1.0;
  publishSphere(header, "goal point", __COUNTER__, pose, color);
}

void Visualization::publishPlanPoint(const std_msgs::Header & header, const tf2::Transform & pose)
{
  std_msgs::ColorRGBA color;
  color.a = 1.0;
  color.g = 0.5;
  color.r = 1.0;
  publishSphere(header, "plan point", __COUNTER__, pose, color);
}

void Visualization::publishSphere(
  const std_msgs::Header & header, const std::string & ns, int id, const tf2::Transform & pose,
  const std_msgs::ColorRGBA & color)
{
  geometry_msgs::Pose msg;
  tf2::toMsg(pose, msg);
  publishSphere(header, ns, id, msg, color);
}

void Visualization::publishSphere(
  const std_msgs::Header & header, const std::string & ns, int id, geometry_msgs::Pose pose,
  const std_msgs::ColorRGBA & color)
{
  visualization_msgs::Marker marker;
  marker.header = header;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose = pose;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color = color;
  marker_pub_.publish(marker);
}

}  // namespace path_tracking_pid
