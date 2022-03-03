#include <geometry_msgs/Pose.h>
#include <ros/publisher.h>
#include <std_msgs/ColorRGBA.h>
#include <tf2/LinearMath/Transform.h>

#include <string>

namespace path_tracking_pid
{

class Visualization
{
public:
  using point_t = geometry_msgs::Point;
  using points_t = std::vector<point_t>;

  explicit Visualization(ros::NodeHandle nh);

  void publishControlPoint(const std_msgs::Header & header, const tf2::Transform & pose);
  void publishAxlePoint(const std_msgs::Header & header, const tf2::Transform & pose);
  void publishGoalPoint(const std_msgs::Header & header, const tf2::Transform & pose);
  void publishPlanPoint(const std_msgs::Header & header, const tf2::Transform & pose);
  void publishCollision(
    const std::string & frame_id, uint8_t cost, const point_t & point, const points_t & footprint,
    const points_t & hull, const points_t & steps, const points_t & path);

private:
  ros::Publisher marker_pub_;
  ros::Publisher collision_marker_pub_;

  void publishSphere(
    const std_msgs::Header & header, const std::string & ns, int id, const tf2::Transform & pose,
    const std_msgs::ColorRGBA & color);
  void publishSphere(
    const std_msgs::Header & header, const std::string & ns, int id,
    const geometry_msgs::Pose & pose, const std_msgs::ColorRGBA & color);
};

}  // namespace path_tracking_pid
