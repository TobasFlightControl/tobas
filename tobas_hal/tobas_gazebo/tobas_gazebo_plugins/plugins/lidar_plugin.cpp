#include <tobas_tools/constants.hpp>

#include "./lidar_plugin.hpp"
#include "../include/tobas_gazebo_plugins/sdfparam.hpp"

using namespace std;
using namespace ignition::math;

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
  nh_.shutdown();
  callback_laser_queue_thread_.join();
}

void GazeboLidarPlugin::Load(sensors::SensorPtr parent, sdf::ElementPtr sdf)
{
  gzmsg << "Loading " << kPluginName << "." << endl;

  RayPlugin::Load(parent, sdf);
  getSdfParams(sdf);

  // Get then name of the parent sensor
  parent_sensor_ = parent;
  parent_ray_sensor_ = dynamic_pointer_cast<sensors::RaySensor>(parent_sensor_);
  if (!parent_ray_sensor_)
  {
    gzthrow(kPluginName << ": Requires a Ray Sensor as its parent.");
  }

  world_ = physics::get_world(parent->WorldName());
  last_update_time_ = world_->SimTime();

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    gzthrow(
      "A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package.");
  }

  // Custom Callback Queue
  ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<sensor_msgs::PointCloud>(
    "/" + ns_ + "/" + tobas::kLidarTopic, 1, boost::bind(&GazeboLidarPlugin::laserConnect, this),
    boost::bind(&GazeboLidarPlugin::laserDisconnect, this), ros::VoidPtr(), &laser_queue_);
  pub_ = nh_.advertise(ao);

  // sensor generation off by default
  parent_ray_sensor_->SetActive(false);

  // start custom queue for laser
  callback_laser_queue_thread_ = boost::thread(boost::bind(&GazeboLidarPlugin::laserQueueThread, this));
}

void GazeboLidarPlugin::onStats(const boost::shared_ptr<msgs::WorldStatistics const>& msg)
{
  sim_time_ = msgs::Convert(msg->sim_time());

  Pose3d pose;
  pose.Pos().X() = 0.5 * sin(0.01 * sim_time_.Double());
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
  common::Time sensor_update_time = parent_sensor_->LastUpdateTime();
  if (sensor_update_time < last_update_time_)
  {
    ROS_WARN_NAMED("block_laser", "Negative sensor update time difference detected.");
    last_update_time_ = sensor_update_time;
  }

  if (last_update_time_ < sensor_update_time)
  {
    putLaserData(sensor_update_time);
    last_update_time_ = sensor_update_time;
  }
}

void GazeboLidarPlugin::getSdfParams(sdf::ElementPtr sdf)
{
  getSdfParam(sdf, "robotNamespace", ns_);
  getSdfParam(sdf, "frameName", frame_name_, kDefaultFrameName);
  getSdfParam(sdf, "gaussianNoiseStddev", noise_stddev_, kDefaultNoiseStddev);
}

void GazeboLidarPlugin::putLaserData(common::Time& update_time)
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

  // set size of cloud message everytime
  cloud_msg_.points.clear();
  cloud_msg_.channels.clear();
  cloud_msg_.channels.push_back(sensor_msgs::ChannelFloat32());

  // Point scan from laser
  boost::mutex::scoped_lock sclock(lock_);

  // Add Frame Name
  cloud_msg_.header.frame_id = frame_name_;
  cloud_msg_.header.stamp.sec = update_time.sec;
  cloud_msg_.header.stamp.nsec = update_time.nsec;

  for (int j = 0; j < ver_range_count; ++j)
  {
    // interpolating in vertical direction
    double vb = (ver_range_count == 1) ? 0 : (double)j * (ver_ray_count - 1) / (ver_range_count - 1);
    const int vja = (int)floor(vb);
    const int vjb = min(vja + 1, ver_ray_count - 1);
    vb = vb - floor(vb);  // fraction from min

    assert(vja >= 0 && vja < ver_ray_count);
    assert(vjb >= 0 && vjb < ver_ray_count);

    for (int i = 0; i < range_count; ++i)
    {
      // Interpolate the range readings from the rays in horizontal direction
      double hb = (range_count == 1) ? 0. : (double)i * (ray_count - 1) / (range_count - 1);
      const int hja = (int)floor(hb);
      const int hjb = min(hja + 1, ray_count - 1);
      hb = hb - floor(hb);  // fraction from min

      assert(hja >= 0 && hja < ray_count);
      assert(hjb >= 0 && hjb < ray_count);

      // indices of 4 corners
      const int j1 = hja + vja * ray_count;
      const int j2 = hjb + vja * ray_count;
      const int j3 = hja + vjb * ray_count;
      const int j4 = hjb + vjb * ray_count;

      // range readings of 4 corners
      const double r1 = parent_ray_sensor_->LaserShape()->GetRange(j1);
      const double r2 = parent_ray_sensor_->LaserShape()->GetRange(j2);
      const double r3 = parent_ray_sensor_->LaserShape()->GetRange(j3);
      const double r4 = parent_ray_sensor_->LaserShape()->GetRange(j4);

      // Range is linear interpolation if values are close, and min if they are very different
      const double r = (1 - vb) * ((1 - hb) * r1 + hb * r2) + vb * ((1 - hb) * r3 + hb * r4);

      // 範囲外もしくは障害物に当たっていない場合はスキップ
      if (r < min_range || max_range - kEpsilonDiff < r)
      {
        continue;
      }

      // Intensity is averaged
      const auto intensity =
        (parent_ray_sensor_->LaserShape()->GetRetro(j1) + parent_ray_sensor_->LaserShape()->GetRetro(j2)
         + parent_ray_sensor_->LaserShape()->GetRetro(j3) + parent_ray_sensor_->LaserShape()->GetRetro(j4))
        / 4;

      // get angles of ray to get xyz for point
      const auto y_angle = (hja + hjb) * y_diff / (ray_count - 1) / 2 + min_angle.Radian();
      const auto p_angle = (vja + vjb) * p_diff / (ver_ray_count - 1) / 2 + ver_min_range.Radian();

      // Point scan from laser
      geometry_msgs::Point32 point;
      // p_angle is rotated by y_angle:
      point.x = r * cos(p_angle) * cos(y_angle);
      point.y = r * cos(p_angle) * sin(y_angle);
      point.z = r * sin(p_angle);

      // add noise to range only if not at max range
      if (max_range - r > kEpsilonDiff)
      {
        point.x += gaussianKernel(0, noise_stddev_);
        point.y += gaussianKernel(0, noise_stddev_);
        point.z += gaussianKernel(0, noise_stddev_);
      }

      cloud_msg_.points.push_back(point);
      cloud_msg_.channels[0].values.push_back(intensity + gaussianKernel(0, noise_stddev_));
    }
  }
  parent_ray_sensor_->SetActive(true);

  // send data out via ros message
  pub_.publish(cloud_msg_);
}

double GazeboLidarPlugin::gaussianKernel(const double& mu, const double& sigma)
{
  // using Box-Muller transform to generate two independent standard normally disbributed normal
  // variables see wikipedia
  const double U = (double)rand() / (double)RAND_MAX;  // normalized uniform random variable
  const double V = (double)rand() / (double)RAND_MAX;  // normalized uniform random variable
  double X = sqrt(-2 * log(U)) * cos(2 * M_PI * V);

  // we will just use X scale to our mu and sigma
  X = sigma * X + mu;
  return X;
}

void GazeboLidarPlugin::laserQueueThread()
{
  while (nh_.ok())
  {
    laser_queue_.callAvailable(ros::WallDuration(kTimeout));
  }
}

GZ_REGISTER_SENSOR_PLUGIN(GazeboLidarPlugin);
}  // namespace gazebo
