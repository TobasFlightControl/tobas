#include "tobas_gazebo_system_plugins/livox_lidar_sensor.hpp"

namespace gazebo
{
LivoxLidarSensor::LivoxLidarSensor()
{
  gzdbg << "constructor" << std::endl;
}

void LivoxLidarSensor::SetScene(gz::rendering::ScenePtr scene)
{
  gzdbg << "set scene function" << std::endl;
  std::lock_guard<std::mutex> lock(this->lidar_mutex_);
  // APIs make it possible for the scene pointer to change
  if (this->Scene() != scene)
  {
    this->removeGpuRays(this->Scene());
    RenderingSensor::SetScene(scene);

    if (this->initialized_)
      this->createLidar();
  }
}

void LivoxLidarSensor::removeGpuRays(gz::rendering::ScenePtr scene)
{
  if (scene)
  {
    scene->DestroySensor(this->gpu_rays_);
  }
  this->gpu_rays_.reset();
  this->gpu_rays_ = nullptr;
}

bool LivoxLidarSensor::Load(const sdf::Sensor &sdf)
{
  gzdbg << "Load function" << std::endl;
  // Load sensor element
  if (!this->Sensor::Load(sdf))
  {
    return false;
  }

  // Check if this is the right type
  if (sdf.Type() != sdf::SensorType::LIDAR &&
      sdf.Type() != sdf::SensorType::GPU_LIDAR)
  {
    TOBAS_ERROR("Attempting to a load a Lidar sensor, but received ", sdf.TypeStr()); // CHECK : このLoad関数がLIDAR用のsdfだけ呼んでくれるかわからない．たくさんエラーが出たらここが原因かもしれない．
  }

  if (sdf.LidarSensor() == nullptr)
  {
    TOBAS_ERROR("Attempting to a load a Lidar sensor, but received a null sensor.");
    return false;
  }

  // Load ray atributes
  this->sdf_lidar_ = *sdf.LidarSensor();
  // Load sampling point number, downsample,  sampling point csv file path for livox lidar sensors
  getSdfParam(sdf.Element(), "samples", this->samples_);
  getSdfParam(sdf.Element(), "downsample", this->downsample_, 1u);
  getSdfParam(sdf.Element(), "csvFilePath", this->csv_file_path_);

  // calculate downsampled data size
  this->downsampled_size_ = this->samples_ / this->downsample_;

  // buffer
  this->lidar_msg_->height = 1;
  this->lidar_msg_->width = this->downsampled_size_;
  this->lidar_msg_->fields.resize(4);
  sensor_msgs::msg::PointField point_field_x;
  point_field_x.name = "x";
  point_field_x.offset = 0;
  point_field_x.datatype = sensor_msgs::msg::PointField::FLOAT32;
  sensor_msgs::msg::PointField point_field_y;
  point_field_y.name = "y";
  point_field_y.offset = sizeof(float);
  point_field_y.datatype = sensor_msgs::msg::PointField::FLOAT32;
  sensor_msgs::msg::PointField point_field_z;
  point_field_z.name = "z";
  point_field_z.offset = sizeof(float) * 2;
  point_field_z.datatype = sensor_msgs::msg::PointField::FLOAT32;
  sensor_msgs::msg::PointField point_field_intensity;
  point_field_intensity.name = "intensity";
  point_field_intensity.offset = sizeof(float) * 3;
  point_field_intensity.datatype = sensor_msgs::msg::PointField::FLOAT32;
  this->lidar_msg_->fields[0] = point_field_x;
  this->lidar_msg_->fields[1] = point_field_y;
  this->lidar_msg_->fields[2] = point_field_z;
  this->lidar_msg_->fields[3] = point_field_intensity;
  this->lidar_msg_->point_step = sizeof(float) * 4;
  this->lidar_msg_->row_step = this->lidar_msg_->width * this->lidar_msg_->point_step;
  this->lidar_msg_->data.resize(this->lidar_msg_->height * this->lidar_msg_->row_step);

  // handle noise model settings
  auto sdf_noise = this->sdf_lidar_.LidarNoise();
  if (sdf_noise.Type() == sdf::NoiseType::GAUSSIAN) {
    // Skip applying noise if gaussian noise params are all 0s
    if (!gz::math::equal(sdf_noise.Mean(), 0.0) ||
        !gz::math::equal(sdf_noise.StdDev(), 0.0) ||
        !gz::math::equal(sdf_noise.BiasMean(), 0.0) ||
        !gz::math::equal(sdf_noise.DynamicBiasStdDev(), 0.0) ||
        !gz::math::equal(sdf_noise.DynamicBiasCorrelationTime(), 0.0))
    {
      this->range_noise_model_ = gz::sensors::NoiseFactory::NewNoiseModel(sdf_noise);
    }
  }
  else if (sdf_noise.Type() != sdf::NoiseType::NONE)
  {
    TOBAS_WARN("The lidar sensor only supports Gaussian noise. The supplied noise type[", static_cast<int>(sdf_noise.Type()), "] is not supported");
  }

  // check the rendering scene and setup lidar
  if (this->Scene())
    this->createLidar();

  // setup ROS point cloud publishers
  this->lidar_pub_ = createPublisher<sensor_msgs::msg::PointCloud2>(tobas::kPointCloud2Topic);

  this->initialized_ = true;
  return true;
}

bool LivoxLidarSensor::Load(const sdf::ElementPtr sdf)
{
  sdf::Sensor sdf_sensor;
  sdf_sensor.Load(sdf);
  return this->Load(sdf_sensor);
}

bool LivoxLidarSensor::Init()
{
  gzdbg << "LidarSensor init" << std::endl;
  return this->Sensor::Init();
}

void LivoxLidarSensor::SetParent(const std::string &parent)
{
  this->Sensor::SetParent(parent);
}

bool LivoxLidarSensor::HasConnections() const
{
  return true; // gazebo messageのconnectionは存在しないが, これをtrueにしないとrendering_sensors_plugin側がこのsensorを起動してくれないので，trueにする
}

bool LivoxLidarSensor::createLidar()
{
  this->gpu_rays_ = this->Scene()->CreateGpuRays();

  if (!this->gpu_rays_)
  {
    TOBAS_ERROR("Unable to create gpu laser sensor\n");
    return false;
  }

  this->gpu_rays_->SetNearClipPlane(this->sdf_lidar_.RangeMin());
  this->gpu_rays_->SetFarClipPlane(this->sdf_lidar_.RangeMax());

  // Mask ranges outside of min/max to +/- inf, as per REP 117
  this->gpu_rays_->SetClamp(false);

  this->gpu_rays_->SetAngleMin(this->sdf_lidar_.HorizontalScanMinAngle().Radian());
  this->gpu_rays_->SetAngleMax(this->sdf_lidar_.HorizontalScanMaxAngle().Radian());

  this->gpu_rays_->SetVerticalAngleMin(this->sdf_lidar_.VerticalScanMinAngle().Radian());
  this->gpu_rays_->SetVerticalAngleMax(this->sdf_lidar_.VerticalScanMaxAngle().Radian());

  this->gpu_rays_->SetRayCount(this->sdf_lidar_.HorizontalScanSamples());
  this->gpu_rays_->SetVerticalRayCount(this->sdf_lidar_.VerticalScanSamples());
  this->gpu_rays_->SetLocalPose(this->Pose());

  this->Scene()->RootVisual()->AddChild(this->gpu_rays_);

  this->gpu_rays_->SetVisibilityMask(this->sdf_lidar_.VisibilityMask());

  this->lidar_frame_connection_ = this->gpu_rays_->ConnectNewGpuRaysFrame(
      std::bind(&LivoxLidarSensor::onNewLidarFrame, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
      std::placeholders::_4, std::placeholders::_5));

  this->AddSensor(this->gpu_rays_);

  return true;
}

void LivoxLidarSensor::onNewLidarFrame(const float *scan,
    unsigned int width, unsigned int height, unsigned int channels,
    const std::string &)
{
  std::lock_guard<std::mutex> lock(this->lidar_mutex_);

  unsigned int samples = width * height * channels;
  unsigned int lidar_buffer_size = samples * sizeof(float);

  if (!this->lidar_buffer_)
    this->lidar_buffer_ = new float[samples];

  memcpy(this->lidar_buffer_, scan, lidar_buffer_size);
}

bool LivoxLidarSensor::Update(const std::chrono::steady_clock::duration &)
{
  if (!this->initialized_)
  {
    TOBAS_ERROR("Not initialized, update ignored.");
    return false;
  }

  if (!this->gpu_rays_)
  {
    TOBAS_ERROR("GpuRays doesn't exist.");
    return false;
  }

  this->Render();

  // Apply noise before publishing the data.
  // this->applyNoise();

  this->publishLidarScan();
  return true;
}

void LivoxLidarSensor::publishLidarScan()
{
  // TODO : fill lidar scan data
  // TODO : add noise
  this->lidar_pub_->publish(std::move(this->lidar_msg_));
}
}  // namespace gazebo
