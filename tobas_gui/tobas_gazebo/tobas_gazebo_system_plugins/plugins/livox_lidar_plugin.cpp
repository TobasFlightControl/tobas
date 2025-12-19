#include <gz/transport/Node.hh>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tobas_constants/constants.hpp>
#include <tobas_ros2_tools/time.hpp>

#include "tobas_gazebo_system_plugins/common/common.hpp"

namespace gazebo
{
/* gazeboのgpu lidarをros2でlivox lidarっぽく使用できるようにbridgeする．
   gpu_lidar sensorとgz-sim-sensors-system pluginにより発出されるgz::msgs::PointCloudPacked型のmessageをsubscribeして，
   ros2のsensor_msgs::msg::PointCloud2型のmessageとして再publishする．*/
class LivoxLidarPlugin : public BaseNode, public gz::sim::System, public gz::sim::ISystemConfigure
{
public:
  explicit LivoxLidarPlugin();

  void Configure(
    const gz::sim::Entity& model,
    const sdf::ElementConstPtr& sdf,
    gz::sim::EntityComponentManager& ecm,
    gz::sim::EventManager& event_mgr) override;

private:
  struct Params
  {
    std::string gpu_ray_topic;
    int gpu_ray_update_rate;
    int update_rate;
    int samples;
  } params_;

  // 1回publishするまでに何回gpu_rayをsamplingするか
  int sampling_times_;
  // gpu_rayから取得したどの点をsamplingするか
  uint sampling_index_ = 0;
  bool initialized_ = false;
  // messageに入っている1つの点の情報にどれくらいのサイズが使われているか
  uint point_step_;
  // samplingする間隔
  uint sampling_interval_;
  // gpu_rayから送られてくるデータに何点sampleが入っているか
  uint gpu_ray_samples_;
  // livox lidarのデータのどのindexのところにデータをつめるか
  uint livox_lidar_data_index = 0;

  // gazebo interfaces
  gz::transport::Node node_;

  // ros2 interfaces
  ros2::PublisherPtr<sensor_msgs::msg::PointCloud2> livox_lidar_publisher_;
  sensor_msgs::msg::PointCloud2::UniquePtr point_cloud_msg_;

  void getSdfParams(const sdf::ElementConstPtr& sdf);
  void gpuRayCb(const gz::msgs::PointCloudPacked& msg);
  void setupPointCloudMsg(const gz::msgs::PointCloudPacked& msg);
};

LivoxLidarPlugin::LivoxLidarPlugin()
{
}

void LivoxLidarPlugin::Configure(
  const gz::sim::Entity&,
  const sdf::ElementConstPtr& sdf,
  gz::sim::EntityComponentManager&,
  gz::sim::EventManager&)
{
  initialize("livox_lidar_plugin", sdf);
  getSdfParams(sdf);

  node_.Subscribe(params_.gpu_ray_topic, &LivoxLidarPlugin::gpuRayCb, this);

  livox_lidar_publisher_ = createPublisher<sensor_msgs::msg::PointCloud2>(tobas::kPointCloud2Topic);
}

void LivoxLidarPlugin::getSdfParams(const sdf::ElementConstPtr& sdf)
{
  getSdfParam(sdf, "gpuRayTopic", params_.gpu_ray_topic);
  getSdfParam(sdf, "gpuRayUpdateRate", params_.gpu_ray_update_rate);
  getSdfParam(sdf, "updateRate", params_.update_rate);
  getSdfParam(sdf, "samples", params_.samples);
  sampling_times_ = params_.gpu_ray_update_rate / params_.update_rate;
}

void LivoxLidarPlugin::gpuRayCb(const gz::msgs::PointCloudPacked& msg)
{
  if (!initialized_) {
    // compute useful parameters
    gpu_ray_samples_ = msg.height() * msg.width();
    sampling_interval_ = gpu_ray_samples_ / params_.samples;
    // samplesが大きすぎるとsampling_interval_ = 0となり破綻する
    if (sampling_interval_ == 0) {
      sampling_interval_ = 1;
    }
    // きちっと割り切れてしまうと一部の点しかsamplingされなくなってしまうので，割り切れなくなるまで上げる
    while (gpu_ray_samples_ % sampling_interval_ == 0) {
      sampling_interval_++;
    }
    point_step_ = msg.point_step();
    initialized_ = true;
  }

  if (!point_cloud_msg_) {
    setupPointCloudMsg(msg);
  }

  // まんべんなくsamplingする
  for (int i = 0; i < params_.samples / sampling_times_; i++) {
    // copy data
    memcpy(
      &point_cloud_msg_->data[livox_lidar_data_index * point_step_],
      &msg.data().c_str()[sampling_index_ * point_step_],
      point_step_);

    // update index where the gpu_ray data will be packed
    sampling_index_ += sampling_interval_;
    if (sampling_index_ >= gpu_ray_samples_) {
      sampling_index_ -= gpu_ray_samples_;
    }

    // update index where the livox_lidar data will be packed
    livox_lidar_data_index += 1;
    if (livox_lidar_data_index >= static_cast<uint>(params_.samples)) {  // sampling completed
      point_cloud_msg_->header.stamp.sec = msg.header().stamp().sec();
      point_cloud_msg_->header.stamp.nanosec = msg.header().stamp().nsec();
      livox_lidar_publisher_->publish(std::move(point_cloud_msg_));
      livox_lidar_data_index = 0;
      return;
    }
  }
}

void LivoxLidarPlugin::setupPointCloudMsg(const gz::msgs::PointCloudPacked& msg)
{
  point_cloud_msg_ = std::make_unique<sensor_msgs::msg::PointCloud2>();
  // fill message field data
  for (int i = 0; i < msg.field_size(); i++) {
    sensor_msgs::msg::PointField field;
    field.name = msg.field(i).name();
    field.count = msg.field(i).count();
    field.offset = msg.field(i).offset();
    switch (msg.field(i).datatype()) {
      default:
      case gz::msgs::PointCloudPacked::Field::INT8:
        field.datatype = sensor_msgs::msg::PointField::INT8;
        break;
      case gz::msgs::PointCloudPacked::Field::UINT8:
        field.datatype = sensor_msgs::msg::PointField::UINT8;
        break;
      case gz::msgs::PointCloudPacked::Field::INT16:
        field.datatype = sensor_msgs::msg::PointField::INT16;
        break;
      case gz::msgs::PointCloudPacked::Field::UINT16:
        field.datatype = sensor_msgs::msg::PointField::UINT16;
        break;
      case gz::msgs::PointCloudPacked::Field::INT32:
        field.datatype = sensor_msgs::msg::PointField::INT32;
        break;
      case gz::msgs::PointCloudPacked::Field::UINT32:
        field.datatype = sensor_msgs::msg::PointField::UINT32;
        break;
      case gz::msgs::PointCloudPacked::Field::FLOAT32:
        field.datatype = sensor_msgs::msg::PointField::FLOAT32;
        break;
      case gz::msgs::PointCloudPacked::Field::FLOAT64:
        field.datatype = sensor_msgs::msg::PointField::FLOAT64;
        break;
    }
    point_cloud_msg_->fields.push_back(field);
  }

  // setup other message data
  point_cloud_msg_->header.frame_id = "map";
  point_cloud_msg_->height = 1;
  point_cloud_msg_->width = params_.samples;
  point_cloud_msg_->is_bigendian = msg.is_bigendian();
  point_cloud_msg_->point_step = msg.point_step();
  point_cloud_msg_->row_step = params_.samples * point_step_;
  point_cloud_msg_->is_dense = msg.is_dense();
  point_cloud_msg_->data.resize(params_.samples * point_step_);
}

}  // namespace gazebo

GZ_ADD_PLUGIN(gazebo::LivoxLidarPlugin, gz::sim::System, gazebo::LivoxLidarPlugin::ISystemConfigure)
