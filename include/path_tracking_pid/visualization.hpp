#include <geometry_msgs/Pose.h>
#include <ros/publisher.h>
#include <std_msgs/ColorRGBA.h>
#include <tf2/LinearMath/Transform.h>

#include <string>
#include <vector>

namespace path_tracking_pid
{
class Visualization
{
public:
  explicit Visualization(ros::NodeHandle nh);

  void publishControlPoint(const std_msgs::Header & header, const tf2::Transform & pose);
  void publishAxlePoint(const std_msgs::Header & header, const tf2::Transform & pose);
  void publishGoalPoint(const std_msgs::Header & header, const tf2::Transform & pose);
  void publishPlanPoint(const std_msgs::Header & header, const tf2::Transform & pose);
  void publishCollisionObject(
    const std_msgs::Header & header, uint8_t cost, const tf2::Vector3 & point);
  void publishExtrapolatedPoses(
    const std_msgs::Header & header, const std::vector<tf2::Vector3> & steps);
  void publishgGoalPosesOnPath(
    const std_msgs::Header & header, const std::vector<tf2::Vector3> & path);
  void publishCollisionFootprint(
    const std_msgs::Header & header, const std::vector<tf2::Vector3> & footprint);
  void publishCollisionPolygon(
    const std_msgs::Header & header, const std::vector<tf2::Vector3> & hull);

private:
  ros::Publisher marker_pub_;

  void publishSphere(
    const std_msgs::Header & header, const std::string & ns, const tf2::Transform & pose,
    const std_msgs::ColorRGBA & color);
  void publishSphere(
    const std_msgs::Header & header, const std::string & ns, const geometry_msgs::Pose & pose,
    const std_msgs::ColorRGBA & color);
};

}  // namespace path_tracking_pid
