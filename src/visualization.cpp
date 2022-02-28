#include <ros/node_handle.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <limits>
#include <path_tracking_pid/visualization.hpp>
#include <string>

namespace path_tracking_pid
{

namespace
{

// Factory function for a visualization scale vector from the given arguments. (Because the
// corresponding type has no appropriate constructor.)
geometry_msgs::Vector3 create_scale(double x, double y, double z)
{
  geometry_msgs::Vector3 result;

  result.x = x;
  result.y = y;
  result.z = z;

  return result;
}

// Factory function for a visualization color from the given arguments. (Because the corresponding
// type has no appropriate constructor.)
std_msgs::ColorRGBA create_color(float r, float g, float b, float a)
{
  std_msgs::ColorRGBA result;

  result.r = r;
  result.g = g;
  result.b = b;
  result.a = a;

  return result;
}

// Factory function for a visualization marker from the given arguments. Some defaults are changed
// to hardcoded values to suit our visualization needs.
visualization_msgs::Marker create_marker(
  const std::string & frame_id, const std::string & ns, std::int32_t type,
  const geometry_msgs::Vector3 & scale, const std_msgs::ColorRGBA & color,
  const Visualization::points_t & points)
{
  visualization_msgs::Marker result;

  result.header.frame_id = frame_id;
  result.header.stamp = ros::Time::now();
  result.ns = ns;
  result.action = visualization_msgs::Marker::ADD;
  result.type = type;
  result.scale = scale;
  result.color = color;
  result.points = points;

  return result;
}

const auto steps_scale = create_scale(0.5, 0.5, 0.0);
const auto steps_color = create_color(1.0, 0.5, 0.0, 1.0);

const auto path_scale = create_scale(0.5, 0.5, 0.0);
const auto path_color = create_color(1.0, 1.0, 0.0, 1.0);

const auto footprint_scale = create_scale(0.1, 0.0, 0.0);
const auto footprint_color = create_color(0.0, 0.0, 1.0, 0.3);

const auto hull_scale = create_scale(0.2, 0.0, 0.0);
const auto hull_color = create_color(1.0, 0.0, 0.0, 0.3);

}  // namespace

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
  const auto marker_steps = create_marker(
    frame_id, "extrapolated poses", visualization_msgs::Marker::POINTS, steps_scale, steps_color,
    steps);

  const auto marker_path = create_marker(
    frame_id, "goal poses on path", visualization_msgs::Marker::POINTS, path_scale, path_color,
    path);

  const auto marker_footprint = create_marker(
    frame_id, "Collision footprint", visualization_msgs::Marker::LINE_LIST, footprint_scale,
    footprint_color, footprint);

  const auto marker_hull = create_marker(
    frame_id, "Collision polygon", visualization_msgs::Marker::LINE_STRIP, hull_scale, hull_color,
    hull);

  auto marker_indicator = create_marker(
    frame_id, "Collision object", visualization_msgs::Marker::CYLINDER,
    create_scale(0.5, 0.5, cost / 255.0),
    create_color(1.0, 0.0, 0.0, static_cast<float>(cost) / 255.0F), {});
  marker_indicator.pose.position = point;
  marker_indicator.pose.position.z = marker_indicator.scale.z * 0.5;
  if (marker_indicator.scale.z <= std::numeric_limits<float>::epsilon()) {
    marker_indicator.action = visualization_msgs::Marker::DELETE;
  }

  visualization_msgs::MarkerArray collision;

  collision.markers.push_back(marker_indicator);
  collision.markers.push_back(marker_footprint);
  collision.markers.push_back(marker_hull);
  if (!marker_steps.points.empty()) {
    collision.markers.push_back(marker_steps);
  }
  if (!marker_path.points.empty()) {
    collision.markers.push_back(marker_path);
  }

  collision_marker_pub_.publish(collision);
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
