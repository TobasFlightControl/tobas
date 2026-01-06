#pragma once

#include <gz/msgs/marker.pb.h>
#include <gz/msgs/marker_v.pb.h>
#include <gz/msgs/world_stats.pb.h>
#include <gz/gui/Plugin.hh>
#include <gz/rendering/Marker.hh>
#include <gz/rendering/Scene.hh>
#include <gz/transport/Node.hh>

namespace gazebo
{
class TobasMarkerManager : public gz::gui::Plugin
{
  Q_OBJECT

  using self = TobasMarkerManager;
  using super = gz::gui::Plugin;

public:
  TobasMarkerManager();

  void LoadConfig(const tinyxml2::XMLElement* _elem) override;

private:
  bool eventFilter(QObject* _obj, QEvent* _event) override;

  void onRender();
  void initialize();

  bool processMarkerMsg(const gz::msgs::Marker& _msg);

  void setVisual(const gz::msgs::Marker& _msg, const gz::rendering::VisualPtr& _visual_ptr);
  void setMarker(const gz::msgs::Marker& _msg, const gz::rendering::MarkerPtr& _marker_ptr);

  gz::rendering::MaterialPtr msgToMaterial(const gz::msgs::Marker& _msg);
  gz::rendering::MarkerType msgToType(const gz::msgs::Marker& _msg);

  void markerCb(const gz::msgs::Marker& _msg);
  void worldStatsCb(const gz::msgs::WorldStatistics& _msg);

  bool listCb(gz::msgs::Marker_V& _rep);

  gz::transport::Node node_;
  gz::rendering::ScenePtr scene_;
  std::mutex mutex_;
  gz::msgs::Marker_Type cur_type_;
  std::list<gz::msgs::Marker> markers_;
  std::map<std::string, std::map<uint64_t, gz::rendering::VisualPtr>> visuals_;
  std::chrono::steady_clock::duration sim_time_;
  std::chrono::steady_clock::duration last_sim_time_;
};
}  // namespace gazebo
