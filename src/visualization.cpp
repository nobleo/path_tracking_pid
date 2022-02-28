#include <ros/node_handle.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <limits>
#include <path_tracking_pid/visualization.hpp>
#include <string>

namespace path_tracking_pid
{
Visualization::Visualization(ros::NodeHandle nh)
: marker_pub_{nh.advertise<visualization_msgs::Marker>("visualization_marker", 4)},
  collision_marker_pub_{nh.advertise<visualization_msgs::MarkerArray>("collision_markers", 3)}
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

void Visualization::publishCollision(
  const std::string & frame_id, uint8_t cost, const point_t & point, const points_t & footprint,
  const points_t & hull, const points_t & steps, const points_t & path)
{
  visualization_msgs::Marker mkSteps;
  mkSteps.header.frame_id = frame_id;
  mkSteps.header.stamp = ros::Time::now();
  mkSteps.ns = "extrapolated poses";
  mkSteps.action = visualization_msgs::Marker::ADD;
  mkSteps.pose.orientation.w = 1.0;
  mkSteps.id = __COUNTER__;
  mkSteps.type = visualization_msgs::Marker::POINTS;
  mkSteps.scale.x = 0.5;
  mkSteps.scale.y = 0.5;
  mkSteps.color.r = 1.0;
  mkSteps.color.g = 0.5;
  mkSteps.color.a = 1.0;
  mkSteps.points = steps;

  visualization_msgs::Marker mkPosesOnPath;
  mkPosesOnPath.header.frame_id = frame_id;
  mkPosesOnPath.header.stamp = ros::Time::now();
  mkPosesOnPath.ns = "goal poses on path";
  mkPosesOnPath.action = visualization_msgs::Marker::ADD;
  mkPosesOnPath.pose.orientation.w = 1.0;
  mkPosesOnPath.id = __COUNTER__;
  mkPosesOnPath.type = visualization_msgs::Marker::POINTS;
  mkPosesOnPath.scale.x = 0.5;
  mkPosesOnPath.scale.y = 0.5;
  mkPosesOnPath.color.r = 1.0;
  mkPosesOnPath.color.g = 1.0;
  mkPosesOnPath.color.a = 1.0;
  mkPosesOnPath.points = path;

  visualization_msgs::Marker mkCollisionFootprint;
  mkCollisionFootprint.header.frame_id = frame_id;
  mkCollisionFootprint.header.stamp = ros::Time::now();
  mkCollisionFootprint.ns = "Collision footprint";
  mkCollisionFootprint.action = visualization_msgs::Marker::ADD;
  mkCollisionFootprint.pose.orientation.w = 1.0;
  mkCollisionFootprint.id = __COUNTER__;
  mkCollisionFootprint.type = visualization_msgs::Marker::LINE_LIST;
  mkCollisionFootprint.scale.x = 0.1;
  mkCollisionFootprint.color.b = 1.0;
  mkCollisionFootprint.color.a = 0.3;
  mkCollisionFootprint.points = footprint;

  visualization_msgs::Marker mkCollisionHull;
  mkCollisionHull.header.frame_id = frame_id;
  mkCollisionHull.header.stamp = ros::Time::now();
  mkCollisionHull.ns = "Collision polygon";
  mkCollisionHull.action = visualization_msgs::Marker::ADD;
  mkCollisionHull.pose.orientation.w = 1.0;
  mkCollisionHull.id = __COUNTER__;
  mkCollisionHull.type = visualization_msgs::Marker::LINE_STRIP;
  mkCollisionHull.scale.x = 0.2;
  mkCollisionHull.color.r = 1.0;
  mkCollisionHull.color.a = 0.3;
  mkCollisionHull.points = hull;

  visualization_msgs::Marker mkCollisionIndicator;
  mkCollisionIndicator.header.frame_id = frame_id;
  mkCollisionIndicator.header.stamp = ros::Time::now();
  mkCollisionIndicator.ns = "Collision object";
  mkCollisionIndicator.pose.orientation.w = 1.0;
  mkCollisionIndicator.id = __COUNTER__;
  mkCollisionIndicator.type = visualization_msgs::Marker::CYLINDER;
  mkCollisionIndicator.scale.x = 0.5;
  mkCollisionIndicator.scale.y = 0.5;
  mkCollisionIndicator.color.r = 1.0;
  mkCollisionIndicator.color.a = 0.0;

  visualization_msgs::MarkerArray mkCollision;
  mkCollisionIndicator.scale.z = cost / 255.0;
  mkCollisionIndicator.color.a = cost / 255.0;
  mkCollisionIndicator.pose.position = point;
  mkCollisionIndicator.pose.position.z = mkCollisionIndicator.scale.z * 0.5;
  if (mkCollisionIndicator.scale.z > std::numeric_limits<float>::epsilon()) {
    mkCollisionIndicator.action = visualization_msgs::Marker::ADD;
  } else {
    mkCollisionIndicator.action = visualization_msgs::Marker::DELETE;
  }

  mkCollision.markers.push_back(mkCollisionIndicator);

  mkCollision.markers.push_back(mkCollisionFootprint);
  mkCollision.markers.push_back(mkCollisionHull);
  if (!mkSteps.points.empty()) {
    mkCollision.markers.push_back(mkSteps);
  }
  if (!mkPosesOnPath.points.empty()) {
    mkCollision.markers.push_back(mkPosesOnPath);
  }

  collision_marker_pub_.publish(mkCollision);
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
