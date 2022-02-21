#include <string>

#include "geometry_msgs/Pose.h"
#include "ros/publisher.h"
#include "std_msgs/ColorRGBA.h"
#include "tf2/LinearMath/Transform.h"

namespace path_tracking_pid
{
class Visualization
{
public:
  Visualization(ros::NodeHandle nh);

  void publishControlPoint(const std_msgs::Header & header, const tf2::Transform & pose);
  void publishAxlePoint(const std_msgs::Header & header, const tf2::Transform & pose);
  void publishGoalPoint(const std_msgs::Header & header, const tf2::Transform & pose);
  void publishPlanPoint(const std_msgs::Header & header, const tf2::Transform & pose);

private:
  void publishSphere(
    const std_msgs::Header & header, const std::string & ns, int id, const tf2::Transform & pose,
    const std_msgs::ColorRGBA & color);
  void publishSphere(
    const std_msgs::Header & header, const std::string & ns, int id, geometry_msgs::Pose pose,
    const std_msgs::ColorRGBA & color);

  ros::Publisher marker_pub_;
};
}  // namespace path_tracking_pid
