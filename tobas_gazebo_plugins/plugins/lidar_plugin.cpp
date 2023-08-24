#include <gazebo_plugins/gazebo_ros_utils.h>

#include "./lidar_plugin.hpp"
#include "../include/tobas_gazebo_plugins/sdfparam.hpp"

using namespace std;

namespace gazebo
{
GazeboLidarPlugin::GazeboLidarPlugin()
{
}

void GazeboLidarPlugin::Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf)
{
  gzmsg << "Loading " << kPluginName << "." << endl;

  super::Load(sensor, sdf);
  getSdfParams(sdf);

  world_ = physics::get_world(sensor->WorldName());
  robot_namespace_ = GetRobotNamespace(sensor, sdf, "Laser");

  parent_ = dynamic_pointer_cast<sensors::RaySensor>(sensor);
  if (!parent_)
    gzthrow(kPluginName << ": Requires a Ray Sensor as its parent.");

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    gzthrow(
      kPluginName << ": A ROS node for Gazebo has not been initialized, unable to load plugin. "
                  << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the "
                     "gazebo_ros package.");
  }

  // ROS callback queue for processing subscription
  deferred_load_thread_ = boost::thread(boost::bind(&GazeboLidarPlugin::loadThread, this));
}

void GazeboLidarPlugin::loadThread()
{
  gazebo_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
  gazebo_node_->Init();

  pmq_.startServiceThread();

  ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<sensor_msgs::PointCloud>(
    topic_name_, 1, boost::bind(&GazeboLidarPlugin::laserConnect, this),
    boost::bind(&GazeboLidarPlugin::laserDisconnect, this), ros::VoidPtr(), NULL);
  pc_pub_ = ros_node_.advertise(ao);
  pc_pub_queue_ = pmq_.addPub<sensor_msgs::PointCloud>();

  // Sensor generation off by default
  parent_->SetActive(false);
}

void GazeboLidarPlugin::getSdfParams(sdf::ElementPtr sdf)
{
  getSdfParam(sdf, "frameName", frame_name_, kDefaultFrameName);
  getSdfParam(sdf, "topicName", topic_name_, kDefaultTopicName);
}

void GazeboLidarPlugin::laserConnect()
{
  if (++laser_connect_count_ == 1)
    laser_sub_ = gazebo_node_->Subscribe(parent_->Topic(), &GazeboLidarPlugin::onScan, this);
}

void GazeboLidarPlugin::laserDisconnect()
{
  if (--laser_connect_count_ == 0)
    laser_sub_.reset();
}

void GazeboLidarPlugin::onScan(ConstLaserScanStampedPtr& msg)
{
  // We got a new message from the Gazebo sensor.
  // Stuff a corresponding ROS message and publish it.
  laser_msg_.header.stamp = ros::Time(msg->time().sec(), msg->time().nsec());
  laser_msg_.header.frame_id = frame_name_;
  laser_msg_.angle_min = msg->scan().angle_min();
  laser_msg_.angle_max = msg->scan().angle_max();
  laser_msg_.angle_increment = msg->scan().angle_step();
  laser_msg_.time_increment = 0;  // Instantaneous simulator scan
  laser_msg_.scan_time = 0;       // FIXME: Not sure whether this is correct
  laser_msg_.range_min = msg->scan().range_min();
  laser_msg_.range_max = msg->scan().range_max();
  laser_msg_.ranges.resize(msg->scan().ranges_size());
  copy(msg->scan().ranges().begin(), msg->scan().ranges().end(), laser_msg_.ranges.begin());
  laser_msg_.intensities.resize(msg->scan().intensities_size());
  copy(
    msg->scan().intensities().begin(), msg->scan().intensities().end(),
    laser_msg_.intensities.begin());

  // Convert from LaserScan to PointCloud
  laser_projector_.projectLaser(laser_msg_, pc_msg_);

  pc_pub_queue_->push(pc_msg_, pc_pub_);
}

GZ_REGISTER_SENSOR_PLUGIN(GazeboLidarPlugin)
}  // namespace gazebo
