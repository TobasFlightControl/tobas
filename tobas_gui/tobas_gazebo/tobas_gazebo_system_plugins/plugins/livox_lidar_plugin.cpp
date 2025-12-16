#include <gz/transport/Node.hh>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tobas_constants/constants.hpp>
#include <tobas_ros2_tools/time.hpp>

#include "tobas_gazebo_system_plugins/common/common.hpp"
#include "tobas_gazebo_system_plugins/rate_manager.hpp"

namespace gazebo
{
/* gazeboのgpu lidarをros2でlivox lidarっぽく使用できるようにbridgeする．
   gpu_lidar sensorとgz-sim-sensors-system pluginにより発出されるgz::msgs::PointCloudPacked型のmessageをsubscribeして，
   ros2のsensor_msgs::msg::PointCloud2型のmessageとして再publishする．*/
class LivoxLidarPlugin : public BaseNode,
                         public gz::sim::System,
                         public gz::sim::ISystemConfigure,
                         public gz::sim::ISystemPostUpdate
{
public:
  explicit LivoxLidarPlugin();
  ~LivoxLidarPlugin();

  void Configure(
    const gz::sim::Entity& model,
    const sdf::ElementConstPtr& sdf,
    gz::sim::EntityComponentManager& ecm,
    gz::sim::EventManager& event_mgr) override;

  void PostUpdate(const gz::sim::UpdateInfo& info, const gz::sim::EntityComponentManager& ecm) override;

private:
  struct Params
  {
    std::string gpu_ray_topic;
    int update_rate;
  } params_;

  RateManager::SharedPtr rate_manager_;

  // gazebo interfaces
  gz::transport::Node node_;

  // ros2 interfaces
  ros2::PublisherPtr<sensor_msgs::msg::PointCloud2> livox_lidar_publisher_;

  void getSdfParams(const sdf::ElementConstPtr& sdf);
  void gpuRayCb(const gz::msgs::PointCloudPacked& msg);
};

LivoxLidarPlugin::LivoxLidarPlugin()
{
}

LivoxLidarPlugin::~LivoxLidarPlugin()
{
  this->executor_->cancel();
  this->spin_thread_.join();
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

  rate_manager_ = std::make_shared<RateManager>(params_.update_rate);

  livox_lidar_publisher_ = createPublisher<sensor_msgs::msg::PointCloud2>(tobas::kPointCloud2Topic);
}

void LivoxLidarPlugin::PostUpdate(const gz::sim::UpdateInfo& info, const gz::sim::EntityComponentManager&)
{
  if (!rate_manager_->update(info.simTime)) {
    return;
  }

  // gzdbg << "before creating message " <<std::endl;
  // auto point_cloud_msg = std::unique_ptr<sensor_msgs::msg::PointCloud2>();
  // ros2::timeChronoToMsg(info.simTime, point_cloud_msg->header.stamp);
  // livox_lidar_publisher_->publish(std::move(point_cloud_msg));
  // gzdbg << "after publishing " <<std::endl;
}

void LivoxLidarPlugin::getSdfParams(const sdf::ElementConstPtr& sdf)
{
  getSdfParam(sdf, "gpuRayTopic", params_.gpu_ray_topic);
  getSdfParam(sdf, "updateRate", params_.update_rate);
}

void LivoxLidarPlugin::gpuRayCb(const gz::msgs::PointCloudPacked& msg)
{
  std::cout << "width : " << msg.width() << std::endl;
  std::cout << "time : " << msg.header().stamp().sec() << ", " << msg.header().stamp().nsec() << std::endl;
}

}  // namespace gazebo

GZ_ADD_PLUGIN(
  gazebo::LivoxLidarPlugin,
  gz::sim::System,
  gazebo::LivoxLidarPlugin::ISystemConfigure,
  gazebo::LivoxLidarPlugin::ISystemPostUpdate)
