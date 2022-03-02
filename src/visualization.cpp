#include <ros/node_handle.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>

#include <algorithm>
#include <limits>
#include <path_tracking_pid/visualization.hpp>
#include <string>
#include <vector>

namespace path_tracking_pid
{
Visualization::Visualization(ros::NodeHandle nh)
: marker_pub_{nh.advertise<visualization_msgs::Marker>("visualization_marker", 10)}
{
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

void Visualization::publishCollisionObject(
  const std_msgs::Header & header, uint8_t cost, const tf2::Vector3 & point)
{
  visualization_msgs::Marker marker;
  marker.header = header;
  marker.ns = "Collision object";
  marker.pose.orientation.w = 1.0;
  marker.id = __COUNTER__;
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.color.r = 1.0;
  marker.color.a = 0.0;
  marker.scale.z = cost / 255.0;
  marker.color.a = cost / 255.0;
  tf2::toMsg(point, marker.pose.position);
  marker.pose.position.z = marker.scale.z * 0.5;
  if (marker.scale.z > std::numeric_limits<float>::epsilon()) {
    marker.action = visualization_msgs::Marker::ADD;
  } else {
    marker.action = visualization_msgs::Marker::DELETE;
  }

  marker_pub_.publish(marker);
}

void Visualization::publishExtrapolatedPoses(
  const std_msgs::Header & header, const std::vector<tf2::Vector3> & steps)
{
  visualization_msgs::Marker marker;
  marker.header = header;
  marker.ns = "extrapolated poses";
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.id = __COUNTER__;
  marker.type = visualization_msgs::Marker::POINTS;
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.color.r = 1.0;
  marker.color.g = 0.5;
  marker.color.a = 1.0;

  std::transform(
    steps.begin(), steps.end(), std::back_inserter(marker.points), [](const auto & step) {
      geometry_msgs::Point p;
      tf2::toMsg(step, p);
      return p;
    });

  marker_pub_.publish(marker);
}

void Visualization::publishgGoalPosesOnPath(
  const std_msgs::Header & header, const std::vector<tf2::Vector3> & path)
{
  visualization_msgs::Marker marker;
  marker.header = header;
  marker.ns = "goal poses on path";
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.id = __COUNTER__;
  marker.type = visualization_msgs::Marker::POINTS;
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.a = 1.0;

  std::transform(
    path.begin(), path.end(), std::back_inserter(marker.points), [](const auto & step) {
      geometry_msgs::Point p;
      tf2::toMsg(step, p);
      return p;
    });

  marker_pub_.publish(marker);
}

void Visualization::publishCollisionFootprint(
  const std_msgs::Header & header, const std::vector<tf2::Vector3> & footprint)
{
  visualization_msgs::Marker marker;
  marker.header = header;
  marker.ns = "Collision footprint";
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.id = __COUNTER__;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.scale.x = 0.1;
  marker.color.b = 1.0;
  marker.color.a = 0.3;

  std::transform(
    footprint.begin(), footprint.end(), std::back_inserter(marker.points), [](const auto & step) {
      geometry_msgs::Point p;
      tf2::toMsg(step, p);
      return p;
    });

  marker_pub_.publish(marker);
}

void Visualization::publishCollisionPolygon(
  const std_msgs::Header & header, const std::vector<tf2::Vector3> & hull)
{
  visualization_msgs::Marker marker;
  marker.header = header;
  marker.ns = "Collision polygon";
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.id = __COUNTER__;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.scale.x = 0.2;
  marker.color.r = 1.0;
  marker.color.a = 0.3;

  std::transform(
    hull.begin(), hull.end(), std::back_inserter(marker.points), [](const auto & step) {
      geometry_msgs::Point p;
      tf2::toMsg(step, p);
      return p;
    });

  marker_pub_.publish(marker);
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
  const std_msgs::Header & header, const std::string & ns, int id, const geometry_msgs::Pose & pose,
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
