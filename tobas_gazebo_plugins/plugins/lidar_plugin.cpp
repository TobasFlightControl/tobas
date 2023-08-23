#include <gazebo/physics/Model.hh>
#include <gazebo/physics/MultiRayShape.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/World.hh>
#include <sensor_msgs/PointCloud.h>

#include "./lidar_plugin.hpp"
#include "../include/tobas_gazebo_plugins/common.hpp"
#include "../include/tobas_gazebo_plugins/sdfparam.hpp"
#include "../include/tobas_gazebo_plugins/ode_multiray_shape.hpp"
#include "../include/tobas_gazebo_plugins/csv_reader.hpp"

using namespace std;
using namespace ignition::math;

namespace gazebo
{
GazeboLidarPlugin::GazeboLidarPlugin()
{
}

void GazeboLidarPlugin::Load(gazebo::sensors::SensorPtr sensor, sdf::ElementPtr sdf)
{
  gzmsg << "Loading " << kPluginName << "." << endl;

  getSdfParams(sdf);

  vector<vector<double>> datas;
  if (!CsvReader::readCsvFile(file_name_, datas))
  {
    gzthrow(kPluginName << ": Failed to get csv file: " << file_name_);
  }

  ray_sensor_ = sensor;

  avia_infos_.clear();
  convertDataToRotateInfo(datas, avia_infos_);
  gzmsg << kPluginName << ": Scan info size is " << avia_infos_.size() << "." << endl;
  max_point_size_ = avia_infos_.size();

  super::Load(sensor, sdf);
  laser_msg_.mutable_scan()->set_frame(sensor->ParentName());
  parent_ = world->EntityByName(sensor->ParentName());
  const auto physics = world->Physics();
  laser_collision_ = physics->CreateCollision("multiray", sensor->ParentName());
  laser_collision_->SetName("ray_sensor_collision");
  laser_collision_->SetRelativePose(sensor->Pose());
  laser_collision_->SetInitialRelativePose(sensor->Pose());
  ray_shape_.reset(new gazebo::physics::OdeMultiRayShape(laser_collision_));
  laser_collision_->SetShape(ray_shape_);

  ray_shape_->rayShapes().reserve(sample_step_ / down_sample_);
  ray_shape_->Load(sdf);
  ray_shape_->Init();

  const auto offset = laser_collision_->RelativePose();
  Vector3d start_point, end_point;
  for (uint32_t j = 0; j < sample_step_; j += down_sample_)
  {
    const uint32_t index = j % max_point_size_;
    const auto& rotate_info = avia_infos_[index];
    Quaterniond ray;
    ray.Euler(Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
    const auto axis = offset.Rot() * ray * Vector3d::UnitX;
    start_point = min_dist_ * axis + offset.Pos();
    end_point = max_dist_ * axis + offset.Pos();
    ray_shape_->AddRay(start_point, end_point);
  }

  registerPublishers();
}

void GazeboLidarPlugin::OnNewLaserScans()
{
  if (ray_shape_ == NULL)
  {
    return;
  }

  vector<pair<int, AviaRotateInfo>> points_pair;
  initializeRays(points_pair, ray_shape_);
  ray_shape_->Update();

  msgs::Set(laser_msg_.mutable_time(), world->SimTime());
  msgs::LaserScan* scan = laser_msg_.mutable_scan();
  initializeScan(scan);

  sensor_msgs::PointCloud scan_point;
  scan_point.header.stamp = ros::Time::now();
  scan_point.header.frame_id = ray_sensor_->Name();

  for (const auto& pair : points_pair)
  {
    auto range = ray_shape_->GetRange(pair.first);
    if (range <= minRange() || maxRange() <= range)
      range = 0;

    const auto rotate_info = pair.second;
    Quaterniond ray;
    ray.Euler(Vector3d(0., rotate_info.zenith, rotate_info.azimuth));

    const auto axis = ray * Vector3d::UnitX;
    const auto point = range * axis;
    scan_point.points.emplace_back();
    scan_point.points.back().x = point.X();
    scan_point.points.back().y = point.Y();
    scan_point.points.back().z = point.Z();
  }
  pc_pub_.publish(scan_point);
  ros::spinOnce();  // FIXME: 不要では？
}

void GazeboLidarPlugin::getSdfParams(sdf::ElementPtr sdf)
{
  getSdfParam(sdf, "csvFileName", file_name_);
  getSdfParam(sdf, "pointCloudTopic", pc_topic_, kDefaultPointCloudTopic);
  getSdfParam(sdf, "samples", sample_step_, NON_NEGATIVE);
  getSdfParam(sdf, "downsample", down_sample_, POSITIVE);

  const auto ray_elem = sdf->GetElement("ray");
  const auto range_elem = ray_elem->GetElement("range");
  getSdfParam(range_elem, "min", min_dist_, kDefaultMinDistance);
  getSdfParam(range_elem, "max", max_dist_, kDefaultMaxDistance);
}

void GazeboLidarPlugin::registerPublishers()
{
  pc_pub_ = nh_.advertise<sensor_msgs::PointCloud>(pc_topic_, 1);
}

void GazeboLidarPlugin::initializeRays(
  vector<pair<int, AviaRotateInfo>>& points_pair,
  boost::shared_ptr<physics::OdeMultiRayShape>& ray_shape)
{
  const auto& rays = ray_shape->rayShapes();
  Vector3d start_point, end_point;
  Quaterniond ray;
  const auto offset = laser_collision_->RelativePose();
  const uint32_t end_index = cur_start_idx_ + sample_step_;
  uint32_t ray_index = 0;
  const auto ray_size = rays.size();
  points_pair.reserve(rays.size());
  for (uint32_t k = cur_start_idx_; k < end_index; k += down_sample_)
  {
    const auto index = k % max_point_size_;
    const auto& rotate_info = avia_infos_[index];
    ray.Euler(Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
    const auto axis = offset.Rot() * ray * Vector3d::UnitX;
    start_point = min_dist_ * axis + offset.Pos();
    end_point = max_dist_ * axis + offset.Pos();
    if (ray_index < ray_size)
    {
      rays[ray_index]->SetPoints(start_point, end_point);
      points_pair.emplace_back(ray_index, rotate_info);
    }
    ++ray_index;
  }
  cur_start_idx_ += sample_step_;
}

void GazeboLidarPlugin::initializeScan(msgs::LaserScan*& scan)
{
  // Store the latest laser scans into laser_msg_
  msgs::Set(scan->mutable_world_pose(), ray_sensor_->Pose() + parent_->WorldPose());
  scan->set_angle_min(minAngle().Radian());
  scan->set_angle_max(maxAngle().Radian());
  scan->set_angle_step(angleResolution());
  scan->set_count(rangeCount());

  scan->set_vertical_angle_min(minVerticalAngle().Radian());
  scan->set_vertical_angle_max(maxVerticalAngle().Radian());
  scan->set_vertical_angle_step(verticalAngleResolution());
  scan->set_vertical_count(verticalRangeCount());

  scan->set_range_min(minRange());
  scan->set_range_max(maxRange());

  scan->clear_ranges();
  scan->clear_intensities();

  for (int j = 0; j < verticalRangeCount(); ++j)
  {
    for (int i = 0; i < rangeCount(); ++i)
    {
      scan->add_ranges(0);
      scan->add_intensities(0);
    }
  }
}

Angle GazeboLidarPlugin::minAngle() const
{
  if (ray_shape_)
    return ray_shape_->MinAngle();
  else
    return -1;
}

Angle GazeboLidarPlugin::maxAngle() const
{
  if (ray_shape_)
    return Angle(ray_shape_->MaxAngle().Radian());
  else
    return -1;
}

Angle GazeboLidarPlugin::minVerticalAngle() const
{
  if (ray_shape_)
    return Angle(ray_shape_->VerticalMinAngle().Radian());
  else
    return -1;
}

Angle GazeboLidarPlugin::maxVerticalAngle() const
{
  if (ray_shape_)
    return Angle(ray_shape_->VerticalMaxAngle().Radian());
  else
    return -1;
}

double GazeboLidarPlugin::minRange() const
{
  if (ray_shape_)
    return ray_shape_->GetMinRange();
  else
    return -1;
}

double GazeboLidarPlugin::maxRange() const
{
  if (ray_shape_)
    return ray_shape_->GetMaxRange();
  else
    return -1;
}

double GazeboLidarPlugin::angleResolution() const
{
  return (maxAngle() - minAngle()).Radian() / (rangeCount() - 1);
}

double GazeboLidarPlugin::rangeResolution() const
{
  if (ray_shape_)
    return ray_shape_->GetResRange();
  else
    return -1;
}

int GazeboLidarPlugin::rayCount() const
{
  if (ray_shape_)
    return ray_shape_->GetSampleCount();
  else
    return -1;
}

int GazeboLidarPlugin::rangeCount() const
{
  if (ray_shape_)
    return ray_shape_->GetSampleCount() * ray_shape_->GetScanResolution();
  else
    return -1;
}

int GazeboLidarPlugin::verticalRayCount() const
{
  if (ray_shape_)
    return ray_shape_->GetVerticalSampleCount();
  else
    return -1;
}

int GazeboLidarPlugin::verticalRangeCount() const
{
  if (ray_shape_)
    return ray_shape_->GetVerticalSampleCount() * ray_shape_->GetVerticalScanResolution();
  else
    return -1;
}

double GazeboLidarPlugin::verticalAngleResolution() const
{
  return (maxVerticalAngle() - minVerticalAngle()).Radian() / (verticalRangeCount() - 1);
}

void GazeboLidarPlugin::convertDataToRotateInfo(
  const vector<vector<double>>& datas,
  vector<AviaRotateInfo>& avia_infos)
{
  avia_infos.reserve(datas.size());
  for (const auto& data : datas)
  {
    if (data.size() != 3)
    {
      gzthrow("data size must be 3.");
    }

    avia_infos.emplace_back();
    avia_infos.back().time = data[0];
    avia_infos.back().azimuth = data[1] * kDegreeToRadian;
    avia_infos.back().zenith = data[2] * kDegreeToRadian - M_PI_2;
  }
}

GZ_REGISTER_SENSOR_PLUGIN(GazeboLidarPlugin)
}  // namespace gazebo
