#pragma once

#include <ros/ros.h>
#include <gazebo/plugins/RayPlugin.hh>

#include "../include/tobas_gazebo_plugins/ode_multiray_shape.hpp"

namespace gazebo
{
// Constants
static const std::string kPluginName = "gps_plugin";

// Default parameters
static const std::string kDefaultPointCloudTopic = "point_cloud";
static constexpr uint32_t kDefaultSamples = 0;
static constexpr uint32_t kDefaultDownSample = 1;
static constexpr double kDefaultMinDistance = 0.1;
static constexpr double kDefaultMaxDistance = 400.;

struct AviaRotateInfo
{
  double time;
  double azimuth;
  double zenith;
};

class GazeboLidarPlugin : public RayPlugin
{
  using super = RayPlugin;

public:
  explicit GazeboLidarPlugin();

  void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf) override;

protected:
  void OnNewLaserScans() override;

private:
  ros::NodeHandle nh_;

  // SDF parameters
  std::string file_name_;
  std::string pc_topic_;
  uint32_t sample_step_;
  uint32_t down_sample_;
  double min_dist_;
  double max_dist_;

  boost::shared_ptr<physics::OdeMultiRayShape> ray_shape_;
  gazebo::physics::CollisionPtr laser_collision_;
  physics::EntityPtr parent_;
  msgs::LaserScanStamped laser_msg_;
  gazebo::sensors::SensorPtr ray_sensor_;
  std::vector<AviaRotateInfo> avia_infos_;

  ros::Publisher pc_pub_;

  uint32_t cur_start_idx_ = 0;
  uint32_t max_point_size_;

  void getSdfParams(sdf::ElementPtr sdf);
  void registerPublishers();

  void initializeRays(
    std::vector<std::pair<int, AviaRotateInfo>>& points_pair,
    boost::shared_ptr<physics::OdeMultiRayShape>& ray_shape);
  void initializeScan(msgs::LaserScan*& scan);

  ignition::math::Angle minAngle() const;
  ignition::math::Angle maxAngle() const;
  ignition::math::Angle minVerticalAngle() const;
  ignition::math::Angle maxVerticalAngle() const;
  double angleResolution() const;
  double verticalAngleResolution() const;
  double minRange() const;
  double maxRange() const;
  double rangeResolution() const;
  int rayCount() const;
  int rangeCount() const;
  int verticalRayCount() const;
  int verticalRangeCount() const;

  static void convertDataToRotateInfo(
    const std::vector<std::vector<double>>& datas,
    std::vector<AviaRotateInfo>& avia_infos);
};
}  // namespace gazebo
