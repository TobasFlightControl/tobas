#include <sensor_msgs/PointCloud.h>

#include <tobas_constants/constants.hpp>

#include "./lidar_plugin.hpp"
#include "../include/tobas_gazebo_plugins/sdfparam.hpp"

using namespace std;
using namespace gz::math;

namespace gazebo
{
GazeboLidarPlugin::GazeboLidarPlugin()
{
}

GazeboLidarPlugin::~GazeboLidarPlugin()
{
  // Finalize the controller / Custom Callback Queue
  laser_queue_.clear();
  laser_queue_.disable();
  node_.shutdown();
  callback_laser_queue_thread_.join();
}

void GazeboLidarPlugin::Load(sensors::SensorPtr parent, sdf::ElementPtr sdf)
{


  RayPlugin::Load(parent, sdf);
  getSdfParams(sdf);

  // Get then name of the parent sensor
  parent_sensor_ = parent;
  parent_ray_sensor_ = dynamic_pointer_cast<sensors::RaySensor>(parent_sensor_);
  if (!parent_ray_sensor_)
    TOBAS_EXIT("Requires a Ray Sensor as its parent.");
  shape_ = parent_ray_sensor_->LaserShape();

  const auto world = physics::get_world(parent->WorldName());
  last_update_time_ = world->SimTime();

  // Custom Callback Queue
  rclcpp::AdvertiseOptions ao = rclcpp::AdvertiseOptions::create<sensor_msgs::msg::PointCloud>(
    path::join(ns(), tobas::kLidarTopic, 1, std::bind(&GazeboLidarPlugin::laserConnect, this),
    std::bind(&GazeboLidarPlugin::laserDisconnect, this), rclcpp::VoidPtr(), &laser_queue_);
  pub_ = createPublisher(ao);

  // sensor generation off by default
  parent_ray_sensor_->SetActive(false);

  // start custom queue for laser
  callback_laser_queue_thread_ = boost::thread(std::bind(&GazeboLidarPlugin::laserQueueThread, this));
}

void GazeboLidarPlugin::onStats(const boost::shared_ptr<msgs::WorldStatistics const>& msg)
{
  sim_time_ = msgs::Convert(msg->sim_time());
}

void GazeboLidarPlugin::laserConnect()
{
  ++laser_connect_count_;
  parent_ray_sensor_->SetActive(true);
}

void GazeboLidarPlugin::laserDisconnect()
{
  if (--laser_connect_count_ == 0)
    parent_ray_sensor_->SetActive(false);
}

void GazeboLidarPlugin::OnNewLaserScans()
{
  const auto sensor_update_time = parent_sensor_->LastUpdateTime();
  if (sensor_update_time < last_update_time_)
  {
    RCLCPP_WARN_NAMED("block_laser", "Negative sensor update time difference detected.");
    last_update_time_ = sensor_update_time;
  }

  if (last_update_time_ < sensor_update_time)
  {
    putLaserData(sensor_update_time);
    last_update_time_ = sensor_update_time;
  }
}

void GazeboLidarPlugin::getSdfParams(const sdf::ElementConstPtr& sdf)
{

  getSdfParam(sdf, "frameName", frame_name_, kDefaultFrameName);
  getSdfParam(sdf, "gaussianNoiseStddev", noise_stddev_, kDefaultNoiseStddev);
}

void GazeboLidarPlugin::putLaserData(const common::Time& update_time)
{
  parent_ray_sensor_->SetActive(false);

  const auto max_angle = parent_ray_sensor_->AngleMax();
  const auto min_angle = parent_ray_sensor_->AngleMin();
  const auto max_range = parent_ray_sensor_->RangeMax();
  const auto min_range = parent_ray_sensor_->RangeMin();
  const auto ray_count = parent_ray_sensor_->RayCount();
  const auto range_count = parent_ray_sensor_->RangeCount();
  const auto ver_ray_count = parent_ray_sensor_->VerticalRayCount();
  const auto ver_range_count = parent_ray_sensor_->VerticalRangeCount();
  const auto ver_max_range = parent_ray_sensor_->VerticalAngleMax();
  const auto ver_min_range = parent_ray_sensor_->VerticalAngleMin();
  const auto y_diff = max_angle.Radian() - min_angle.Radian();
  const auto p_diff = ver_max_range.Radian() - ver_min_range.Radian();

  // Create a cloud message
  const auto cloud_msg =std::make_unique<sensor_msgs::msg::PointCloud>();

  // Set size of cloud message everytime
  cloud_msg->channels.push_back(sensor_msgs::msg::ChannelFloat32());

  // Point scan from laser
  boost::mutex::scoped_lock sclock(lock_);

  // Add Frame Name
  cloud_msg->header.frame_id = frame_name_;
  cloud_msg->header.stamp.sec = update_time.sec;
  cloud_msg->header.stamp.nsec = update_time.nsec;

  for (int j = 0; j < ver_range_count; ++j)
  {
    // Interpolating in vertical direction
    auto vb = (ver_range_count == 1) ? 0. : (double)j * (ver_ray_count - 1) / (ver_range_count - 1);
    const auto vja = (int)floor(vb);
    const auto vjb = min(vja + 1, ver_ray_count - 1);
    vb = vb - floor(vb);  // Fraction from min

    assert(vja >= 0 && vja < ver_ray_count);
    assert(vjb >= 0 && vjb < ver_ray_count);

    for (int i = 0; i < range_count; ++i)
    {
      // Interpolate the range readings from the rays in horizontal direction
      auto hb = (range_count == 1) ? 0. : (double)i * (ray_count - 1) / (range_count - 1);
      const auto hja = (int)floor(hb);
      const auto hjb = min(hja + 1, ray_count - 1);
      hb = hb - floor(hb);  // Fraction from min

      assert(hja >= 0 && hja < ray_count);
      assert(hjb >= 0 && hjb < ray_count);

      // Indices of 4 corners
      const auto j1 = hja + vja * ray_count;
      const auto j2 = hjb + vja * ray_count;
      const auto j3 = hja + vjb * ray_count;
      const auto j4 = hjb + vjb * ray_count;

      // Range readings of 4 corners
      const auto r1 = shape_->GetRange(j1);
      const auto r2 = shape_->GetRange(j2);
      const auto r3 = shape_->GetRange(j3);
      const auto r4 = shape_->GetRange(j4);

      // Range is linear interpolation if values are close, and min if they are very different
      const auto r = (1 - vb) * ((1 - hb) * r1 + hb * r2) + vb * ((1 - hb) * r3 + hb * r4);

      // 範囲外もしくは障害物に当たっていない場合はスキップ
      if (r < min_range || max_range - kEpsilonDiff < r)
        continue;

      // Intensity is averaged
      const auto intensity =
        (shape_->GetRetro(j1) + shape_->GetRetro(j2) + shape_->GetRetro(j3) + shape_->GetRetro(j4)) / 4;

      // Get angles of ray to get xyz for point
      const auto y_angle = (hja + hjb) * y_diff / (ray_count - 1) / 2 + min_angle.Radian();
      const auto p_angle = (vja + vjb) * p_diff / (ver_ray_count - 1) / 2 + ver_min_range.Radian();

      // Point scan from laser
      geometry_msgs::msg::Point32 point;
      // p_angle is rotated by y_angle:
      point.x = r * cos(p_angle) * cos(y_angle);
      point.y = r * cos(p_angle) * sin(y_angle);
      point.z = r * sin(p_angle);

      // Add noise to range only if not at max range
      if (max_range - r > kEpsilonDiff)
      {
        point.x += gaussianKernel(0, noise_stddev_);
        point.y += gaussianKernel(0, noise_stddev_);
        point.z += gaussianKernel(0, noise_stddev_);
      }

      cloud_msg->points.push_back(point);
      cloud_msg->channels[0].values.push_back(intensity + gaussianKernel(0, noise_stddev_));
    }
  }
  parent_ray_sensor_->SetActive(true);

  // send data out via ros message
  pub_->publish(cloud_msg);
}

double GazeboLidarPlugin::gaussianKernel(const double& mu, const double& sigma)
{
  // using Box-Muller transform to generate two independent standard normally disbributed normal
  // variables see wikipedia
  const auto U = (double)rand() / (double)RAND_MAX;  // normalized uniform random variable
  const auto V = (double)rand() / (double)RAND_MAX;  // normalized uniform random variable
  auto X = sqrt(-2 * log(U)) * cos(2 * M_PI * V);

  // we will just use X scale to our mu and sigma
  X = sigma * X + mu;
  return X;
}

void GazeboLidarPlugin::laserQueueThread()
{
  while (node_.ok())
    laser_queue_.callAvailable(rclcpp::WallDuration(kTimeout));
}

GZ_REGISTER_SENSOR_PLUGIN(GazeboLidarPlugin);
}  // namespace gazebo
