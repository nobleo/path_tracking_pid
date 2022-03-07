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
// Factory function for a visualization color from the given arguments. (Because the corresponding
// type has no appropriate constructor.)
std_msgs::ColorRGBA create_color(float r, float g, float b, float a = 1)
{
  std_msgs::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

const auto red = create_color(1, 0, 0);
const auto green = create_color(0, 1, 0);
const auto blue = create_color(1, 0, 1);
const auto yellow = create_color(1, 1, 0);
const auto orange = create_color(1, 0.5, 0);

std::vector<geometry_msgs::Point> to_msg(std::vector<tf2::Vector3> points)
{
  std::vector<geometry_msgs::Point> msgs;
  std::transform(points.begin(), points.end(), std::back_inserter(msgs), [](const auto & msg) {
    geometry_msgs::Point p;
    tf2::toMsg(msg, p);
    return p;
  });
  return msgs;
}

Visualization::Visualization(ros::NodeHandle nh)
: marker_pub_{nh.advertise<visualization_msgs::Marker>("visualization_marker", 10)}
{
}

void Visualization::publishControlPoint(
  const std_msgs::Header & header, const tf2::Transform & pose)
{
  publishSphere(header, "control point", pose, green);
}

void Visualization::publishAxlePoint(const std_msgs::Header & header, const tf2::Transform & pose)
{
  std_msgs::ColorRGBA color;
  publishSphere(header, "axle point", pose, blue);
}

void Visualization::publishGoalPoint(const std_msgs::Header & header, const tf2::Transform & pose)
{
  std_msgs::ColorRGBA color;
  publishSphere(header, "goal point", pose, red);
}

void Visualization::publishPlanPoint(const std_msgs::Header & header, const tf2::Transform & pose)
{
  publishSphere(header, "plan point", pose, orange);
}

void Visualization::publishCollisionObject(
  const std_msgs::Header & header, uint8_t cost, const tf2::Vector3 & point)
{
  visualization_msgs::Marker marker;
  marker.header = header;
  marker.ns = "Collision object";
  marker.pose.orientation.w = 1.0;
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.color = red;
  marker.color.a = cost / 255.0;
  marker.scale.z = cost / 255.0;
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
  marker.type = visualization_msgs::Marker::POINTS;
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.color = orange;
  marker.points = to_msg(steps);
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
  marker.type = visualization_msgs::Marker::POINTS;
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.color = yellow;
  marker.points = to_msg(path);
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
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.scale.x = 0.1;
  marker.color = blue;
  marker.color.a = 0.3;
  marker.points = to_msg(footprint);
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
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.scale.x = 0.2;
  marker.color = red;
  marker.color.a = 0.3;
  marker.points = to_msg(hull);
  marker_pub_.publish(marker);
}

void Visualization::publishSphere(
  const std_msgs::Header & header, const std::string & ns, const tf2::Transform & pose,
  const std_msgs::ColorRGBA & color)
{
  geometry_msgs::Pose msg;
  tf2::toMsg(pose, msg);
  publishSphere(header, ns, msg, color);
}

void Visualization::publishSphere(
  const std_msgs::Header & header, const std::string & ns, const geometry_msgs::Pose & pose,
  const std_msgs::ColorRGBA & color)
{
  visualization_msgs::Marker marker;
  marker.header = header;
  marker.ns = ns;
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
